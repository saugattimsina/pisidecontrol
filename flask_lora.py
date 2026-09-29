"""
Flask LoRa ground-station relay for the pilora dashboard.

Two radio backends:
  sx1262 - SX1262 wired to a Raspberry Pi over SPI. Radio settings, framing and
           reassembly are a mirror of the drone's src/lora_link.cpp - keep the
           two in step (see the "LoRa link" block below).
  serial - a USB/UART LoRa module on /dev/ttyUSB0 (transparent bridge)

LORA_BACKEND=auto (default) picks sx1262 when LoRaRF is installed and SPI is
enabled, otherwise serial. Override with environment variables, e.g.
  LORA_BACKEND=sx1262 python flask_lora.py
  LORA_BACKEND=serial LORA_PORT=/dev/ttyACM0 python flask_lora.py

Only ONE program can own the radio: stop lora-c.py / lora_serial.py /
ground_station.py before starting this. LORA_DEBUG=1 prints every raw packet.
"""
import json
import os
import queue
import re
import threading
import time
import urllib.parse
import urllib.request
from collections import deque
from datetime import datetime, timezone

from flask import Flask, request, jsonify, render_template

app = Flask(__name__)


@app.route('/')
def index():
    return render_template('index.html')


# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
LORA_BACKEND = os.environ.get('LORA_BACKEND', 'auto').lower()

# serial backend
SERIAL_PORT = os.environ.get('LORA_PORT', '/dev/ttyUSB0')
BAUD_RATE = int(os.environ.get('LORA_BAUD', '115200'))

# ----------------------------------------------------------------------------
# LoRa link - MUST match the drone (include/lora_link.hpp LoraConfig and
# src/lora_link.cpp). Change a value here -> change it there too.
# ----------------------------------------------------------------------------
SPI_BUS, SPI_CS = 0, 0                                               # spi_device /dev/spidev0.0
PIN_RESET, PIN_BUSY, PIN_IRQ, PIN_TXEN, PIN_RXEN = 22, 23, 24, 5, 6  # pin_reset/busy/irq/txen/rxen
FREQUENCY = 915_000_000       # frequency
TX_POWER = 22                 # tx_power (dBm)
SPREADING_FACTOR = 9          # spreading_factor
BANDWIDTH = 125_000           # bandwidth (Hz)
CODING_RATE = 5               # coding_rate: 5 = 4/5
PREAMBLE_LEN = 12             # preamble_len
SYNC_WORD = 0x1424            # sync_word, the raw register value (private network).
                              # LoRaRF writes values > 0xFF as-is; 0x12 is its short form.
# Low-data-rate optimisation: same rule as SX1262::begin() (on when a symbol > 16 ms).
LDRO = (2 ** SPREADING_FACTOR) * 1000.0 / BANDWIDTH > 16.0
# Packets: explicit header, CRC on, standard IQ (setPacketParams); RX takes up to 255 bytes.
MAX_PACKET = 255              # SX1262::transmit() cuts longer messages at 255 bytes
TX_TIMEOUT_S = 5.0            # SX1262::transmit() deadline (255 B at SF9/125k is ~1.3 s)
# Receive-side reassembly - LoraLink::radioLoop():
REASSEMBLY_GAP_S = 0.3        # kReassemblyGap: no newline after this much silence -> flush
MAX_MESSAGE = 512             # kMaxMessage: buffer longer than this -> discard
RX_POLL_S = 0.005             # radioLoop sleeps 5 ms per pass
# Addressing - LoraLink::handlePacket():
#   "<id>-<cmd>"  only that drone runs it; ACK (drone run with --lora-ack) is "ACK <id> <cmd>"
#   "all-<cmd>"   every drone runs it, ONLY land / l / stop / estop / rtl, never ACKed
ALL_PREFIX = "all-"
BROADCAST_COMMANDS = {"land", "l", "stop", "estop", "rtl"}
LORA_DEBUG = os.environ.get('LORA_DEBUG', '0') == '1'   # print every raw fragment

# How long to wait for the drone's reply (ACK + telemetry can be two packets).
MAX_WAIT_SECONDS = 4.0

# Commands that must never queue behind a background poll (the drone's
# processCommand() also lets these bypass its command queue).
EMERGENCY_COMMANDS = {"land", "l", "stop", "estop"}

# Commands whose real answer is a telemetry line sent after any ACK.
# Every other command returns as soon as its ACK arrives.
TELEMETRY_COMMANDS = {"status", "sensors", "ping"}


# ----------------------------------------------------------------------------
# Telemetry JSON
#
# Every line the drone sends ("[Drone 1] ping: PONG|A:Y|M:GUIDED|Alt:20.1m|...")
# is parsed into a per-drone JSON state, served at GET /telemetry and, if
# TELEMETRY_PUSH_URL is set, POSTed to your display server on every update.
#   TELEMETRY_PUSH_URL=https://...    (default below; set to "" to disable)
#   TELEMETRY_PUSH_TOKEN=secret       (optional, sent as Bearer)
# ----------------------------------------------------------------------------
TELEMETRY_PUSH_URL = os.environ.get('TELEMETRY_PUSH_URL', 'https://api.synaix.viclyx.com/api/status/pi-data').strip()
TELEMETRY_PUSH_TOKEN = os.environ.get('TELEMETRY_PUSH_TOKEN', '').strip()
ONLINE_TIMEOUT_S = 10.0          # no message for this long -> "online": false

_DRONE_RE = re.compile(r'^\[Drone ([^\]]+)\]\s*(.*)$')
_TELEMETRY_KEYS = {"A", "M", "Alt", "Baro", "Spd", "Vz", "Sats", "Loc", "GPS", "Trk", "Go"}

telemetry_lock = threading.Lock()
drones = {}                      # drone_id -> state dict
events = deque(maxlen=100)       # non-telemetry replies ("arm: OK armed", "gys: ARRIVED"...)
_push_queue = queue.Queue(maxsize=50)


def _now_iso():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def _num(text, suffix=""):
    try:
        return float(text[:-len(suffix)] if suffix and text.endswith(suffix) else text)
    except (TypeError, ValueError):
        return None


def _empty_state(drone_id):
    return {
        "drone_id": drone_id,
        "updated_at": None,
        "armed": None,
        "mode": None,
        "altitude_m": None,
        "baro_altitude_m": None,
        "ground_speed_ms": None,
        "climb_rate_ms": None,
        "gps": {"enabled": None, "satellites": None, "fix": None, "lat": None, "lon": None},
        "tracking": None,
        "goto": {"active": False, "remaining_m": None},
        "link": {"rssi_dbm": None, "snr_db": None},
        "last_reply": None,
        "raw": None,
    }


def parse_drone_line(line):
    """'[Drone 1] ping: PONG|A:Y|Alt:3.4m' -> ('1', 'ping', 'PONG', {'A':'Y','Alt':'3.4m'}).
    Returns None for lines that don't come from a drone."""
    m = _DRONE_RE.match(line.strip())
    if not m:
        return None
    drone_id, body = m.group(1).strip(), m.group(2)
    parts = body.split('|')
    keyword, text, fields = None, None, {}
    first = parts[0]
    k0 = first.split(':', 1)[0].strip()
    if ':' in first and k0 not in _TELEMETRY_KEYS:
        keyword, text = [x.strip() for x in first.split(':', 1)]
        parts = parts[1:]
    for p in parts:
        if ':' in p:
            k, v = p.split(':', 1)
            fields[k.strip()] = v.strip()
    return drone_id, keyword, text, fields


def record_line(line, rssi=None, snr=None):
    """Update the JSON state from one received line. Safe to call from any thread."""
    parsed = parse_drone_line(line)
    if parsed is None:
        return
    drone_id, keyword, text, f = parsed
    now = _now_iso()
    with telemetry_lock:
        st = drones.setdefault(drone_id, _empty_state(drone_id))
        st["updated_at"] = now
        st["raw"] = line
        st["link"] = {"rssi_dbm": rssi, "snr_db": snr}
        if keyword:
            st["last_reply"] = {"command": keyword, "text": text, "at": now}
        if "A" in f:    st["armed"] = f["A"] == "Y"
        if "M" in f:    st["mode"] = f["M"]
        if "Alt" in f:  st["altitude_m"] = _num(f["Alt"], "m")
        if "Baro" in f: st["baro_altitude_m"] = _num(f["Baro"], "m")
        if "Spd" in f:  st["ground_speed_ms"] = _num(f["Spd"])
        if "Vz" in f:   st["climb_rate_ms"] = _num(f["Vz"])
        if "Trk" in f:  st["tracking"] = f["Trk"] == "Y"
        gps = st["gps"]
        if f.get("GPS") == "OFF":
            st["gps"] = {"enabled": False, "satellites": None, "fix": False, "lat": None, "lon": None}
        elif "Sats" in f or "Loc" in f:
            gps["enabled"] = True
            if "Sats" in f:
                sats = _num(f["Sats"])
                gps["satellites"] = int(sats) if sats is not None else None
            if "Loc" in f:
                if f["Loc"] == "none" or ',' not in f["Loc"]:
                    gps.update(fix=False, lat=None, lon=None)
                else:
                    la, lo = f["Loc"].split(',', 1)
                    lat, lon = _num(la), _num(lo)
                    ok = lat is not None and lon is not None and not (lat == 0 and lon == 0)
                    gps.update(fix=ok, lat=lat if ok else None, lon=lon if ok else None)
        if f:   # a telemetry line: Go present means a gys go-to is running
            st["goto"] = {"active": "Go" in f, "remaining_m": _num(f["Go"], "m") if "Go" in f else None}
        if keyword and not f:
            events.append({"at": now, "drone_id": drone_id, "command": keyword, "text": text})
        if keyword == "gys" and text and text.upper().startswith("BALLOON SEEN"):
            _chasing_until[drone_id] = time.time() + TARGET_HOLDOFF_S   # its own camera has it now
        if keyword == "gys" and text and not text.startswith("OK"):
            st["goto"] = {"active": False, "remaining_m": None}   # ARRIVED / BALLOON SEEN / TIMEOUT
        snapshot = json.loads(json.dumps(st))
    _queue_push(snapshot)
    _publish_telemetry(snapshot)     # to that drone's own WebSocket (DRONE-<n>), if connected


def _with_online(st):
    out = json.loads(json.dumps(st))
    age = None
    if st["updated_at"]:
        age = (datetime.now(timezone.utc) - datetime.fromisoformat(st["updated_at"])).total_seconds()
    out["age_s"] = round(age, 1) if age is not None else None
    out["online"] = age is not None and age < ONLINE_TIMEOUT_S
    return out


def _queue_push(snapshot):
    if not TELEMETRY_PUSH_URL:
        return
    try:
        _push_queue.put_nowait(snapshot)
    except queue.Full:
        pass   # display server slow/down: drop rather than block the radio


def _push_worker():
    ok_state = None            # log only when push starts working / starts failing
    last_err_print = 0.0
    print(f"[*] Pushing telemetry JSON to {TELEMETRY_PUSH_URL}", flush=True)
    while True:
        snapshot = _push_queue.get()
        # Sent as a form POST with one parameter:  data=<telemetry JSON string>
        # (server side: $_POST['data'] in PHP, request.form['data'] in Flask,
        #  req.body.data with express.urlencoded() in Node) - then JSON-decode it.
        body = urllib.parse.urlencode({"data": json.dumps(_with_online(snapshot))}).encode()
        headers = {"Content-Type": "application/x-www-form-urlencoded"}
        if TELEMETRY_PUSH_TOKEN:
            headers["Authorization"] = f"Bearer {TELEMETRY_PUSH_TOKEN}"
        try:
            req = urllib.request.Request(TELEMETRY_PUSH_URL, data=body, headers=headers, method="POST")
            with urllib.request.urlopen(req, timeout=3) as resp:
                resp.read()
                if ok_state is not True:
                    print(f"[*] Telemetry push OK (HTTP {resp.status})", flush=True)
                ok_state = True
        except Exception as e:
            detail = e
            if hasattr(e, "read"):
                try:
                    detail = f"{e} - {e.read(200).decode(errors='replace')}"
                except Exception:
                    pass
            if ok_state is not False or time.time() - last_err_print > 30:
                print(f"[!] Telemetry push to {TELEMETRY_PUSH_URL} failed: {detail}", flush=True)
                last_err_print = time.time()
            ok_state = False


if TELEMETRY_PUSH_URL:
    threading.Thread(target=_push_worker, daemon=True).start()


@app.route('/telemetry')
def telemetry_all():
    """All drones + recent command replies, e.g. for a display server to poll."""
    with telemetry_lock:
        return jsonify({
            "server_time": _now_iso(),
            "drones": {k: _with_online(v) for k, v in drones.items()},
            "events": list(events)[-50:],
        })


@app.route('/telemetry/<drone_id>')
def telemetry_one(drone_id):
    with telemetry_lock:
        st = drones.get(drone_id)
        if st is None:
            return jsonify({"error": f"no data from drone {drone_id} yet"}), 404
        return jsonify(_with_online(st))


# ----------------------------------------------------------------------------
# Radio backends. Both hand every complete received line to _deliver().
# ----------------------------------------------------------------------------
def _deliver(radio_obj, line, rssi=None, snr=None):
    """One complete line from the drone: log it, update telemetry, wake the waiting command."""
    sig = f"  (RSSI {rssi} dBm, SNR {snr} dB)" if rssi is not None else ""
    print(f"[RX] {line}{sig}", flush=True)
    record_line(line, rssi, snr)
    radio_obj.rx_queue.put(line)


class SerialRadio:
    """USB/UART LoRa module (transparent bridge): the module does the radio part,
    lines are newline-framed exactly like the SPI link."""
    name = "serial"

    def __init__(self):
        import serial  # pyserial
        self.rx_queue = queue.Queue()
        self._tx_lock = threading.Lock()
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        time.sleep(2.0)                     # ESP32-style boards reboot on open
        self.ser.reset_input_buffer()
        self.alive = True
        threading.Thread(target=self._rx_loop, daemon=True).start()
        print(f"[*] Serial LoRa module on {SERIAL_PORT} at {BAUD_RATE} baud.")

    def _rx_loop(self):
        while self.alive:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
            except Exception as e:
                print(f"[!] Serial read error: {e}", flush=True)
                self.alive = False
                break
            if line:
                _deliver(self, line)

    def send(self, line):
        with self._tx_lock:
            self.ser.write((line.rstrip("\r\n") + "\n").encode('utf-8'))
            self.ser.flush()


class LineAssembler:
    """Joins received LoRa packets into lines with the SAME rules as the drone's
    LoraLink::radioLoop() (src/lora_link.cpp). A DX-LR30 module splits one line into
    ~5-byte packets ("1-sta" + "tus\\n"), so a line is only used once it is whole:
      - packets are appended; everything up to the LAST newline is complete and handed on,
        whatever follows it waits for more packets
      - no newline and REASSEMBLY_GAP_S of silence -> the leftover is handed on anyway
      - a packet with a CRC error "poisons" the message it belongs to: that whole message
        is dropped (a missing piece could turn "1-t 15" into "1-t 1")
      - more than MAX_MESSAGE bytes buffered -> discarded
    Pure logic, no radio - tested on its own."""

    def __init__(self, on_line):
        self.on_line = on_line
        self.buf = ""
        self.poisoned = False
        self.last = 0.0

    def fragment(self, text, now):
        if LORA_DEBUG:
            print(f'[LORA] RX fragment "{text.strip()}"', flush=True)
        self.buf += text
        self.last = now
        if len(self.buf) > MAX_MESSAGE:
            print("[LORA] ⚠️ Receive buffer overflow - discarding.", flush=True)
            self.buf, self.poisoned = "", False
            return
        nl = max(self.buf.rfind("\n"), self.buf.rfind("\r"))
        if nl < 0:
            return
        complete, self.buf = self.buf[:nl + 1], self.buf[nl + 1:]
        if self.poisoned:
            print(f'[LORA] ⚠️ Dropped message "{complete.strip()}" (a packet was corrupted)', flush=True)
            self.poisoned = False
            return
        self._emit(complete)

    def corrupted(self, now):
        self.poisoned = True
        self.last = now

    def tick(self, now):
        """Call often: flushes a message whose packets stopped without a newline."""
        if (self.buf or self.poisoned) and now - self.last > REASSEMBLY_GAP_S:
            if self.poisoned:
                print(f'[LORA] ⚠️ Dropped incomplete message "{self.buf.strip()}" '
                      "(a packet was corrupted)", flush=True)
            elif self.buf.strip():
                self._emit(self.buf)
            self.buf, self.poisoned = "", False

    def _emit(self, text):
        for line in re.split(r"[\r\n]", text):       # one packet may hold several lines
            line = line.strip()
            if line:
                self.on_line(line)


class Sx1262Radio:
    """SX1262 on SPI through LoRaRF, configured like the drone's SX1262::begin()."""
    name = "sx1262"

    def __init__(self):
        from LoRaRF import SX126x
        self.rx_queue = queue.Queue()
        self._lock = threading.Lock()       # one SPI user at a time
        self._rssi = self._snr = None
        self._rx = LineAssembler(lambda line: _deliver(self, line, self._rssi, self._snr))
        LoRa = SX126x()
        if not LoRa.begin(SPI_BUS, SPI_CS, PIN_RESET, PIN_BUSY, PIN_IRQ, PIN_TXEN, PIN_RXEN):
            raise RuntimeError("SX1262 init failed - check wiring / SPI enabled / "
                               "the drone program or lora-c.py not holding the pins")
        LoRa.setFrequency(FREQUENCY)
        LoRa.setTxPower(TX_POWER, LoRa.TX_POWER_SX1262)
        LoRa.setLoRaModulation(SPREADING_FACTOR, BANDWIDTH, CODING_RATE, LDRO)
        LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, PREAMBLE_LEN, MAX_PACKET, crcType=True, invertIq=False)
        LoRa.setSyncWord(SYNC_WORD)
        self.LoRa = LoRa
        self.ST_RX_DONE = getattr(LoRa, "STATUS_RX_DONE", 7)
        self.ST_HEADER_ERR = getattr(LoRa, "STATUS_HEADER_ERR", 8)
        self.ST_CRC_ERR = getattr(LoRa, "STATUS_CRC_ERR", 9)
        self.alive = True
        with self._lock:
            LoRa.request(LoRa.RX_CONTINUOUS)
        threading.Thread(target=self._rx_loop, daemon=True).start()
        print(f"[LORA] ✅ Radio ready | {FREQUENCY / 1e6:.1f} MHz | SF{SPREADING_FACTOR} | "
              f"BW {BANDWIDTH / 1e3:.1f} kHz | CR 4/{CODING_RATE} | {TX_POWER} dBm | "
              f"sync 0x{SYNC_WORD:04X}", flush=True)

    def _rx_loop(self):
        LoRa = self.LoRa
        while self.alive:
            frag, status = None, None
            try:
                with self._lock:
                    status = LoRa.status()           # reading it clears the event (RX continuous)
                    if status == self.ST_RX_DONE:
                        n = LoRa.available()
                        frag = bytes(LoRa.read(n)).decode(errors="replace") if n else ""
                    if status in (self.ST_RX_DONE, self.ST_CRC_ERR):
                        self._rssi, self._snr = LoRa.packetRssi(), LoRa.snr()
                    if status in (self.ST_RX_DONE, self.ST_CRC_ERR, self.ST_HEADER_ERR):
                        LoRa.request(LoRa.RX_CONTINUOUS)   # no-op if still receiving
            except Exception as e:
                print(f"[!] SX1262 RX error: {e}", flush=True)
                time.sleep(0.5)

            now = time.time()
            if status == self.ST_HEADER_ERR:
                print(f"[LORA] ⚠️ Heard a LoRa signal but its header didn't decode - check that the "
                      f"drone uses SF{SPREADING_FACTOR} / BW {BANDWIDTH / 1e3:.1f} kHz / "
                      f"CR 4/{CODING_RATE}", flush=True)
            elif status == self.ST_CRC_ERR:
                print(f"[LORA] ⚠️ Dropped corrupted packet (CRC error, RSSI {self._rssi} dBm, "
                      f"SNR {self._snr} dB)", flush=True)
                self._rx.corrupted(now)
            elif frag:
                self._rx.fragment(frag, now)
            self._rx.tick(now)
            time.sleep(RX_POLL_S)

    def send(self, line):
        """Like LoraLink::send() + SX1262::transmit(): one packet ending in '\\n' (the other
        side waits for it), at most MAX_PACKET bytes, then straight back to continuous RX."""
        data = list((line.rstrip("\r\n") + "\n").encode()[:MAX_PACKET])
        with self._lock:
            LoRa = self.LoRa
            LoRa.beginPacket()
            LoRa.write(data, len(data))
            LoRa.endPacket()
            done = LoRa.wait(TX_TIMEOUT_S)
            LoRa.request(LoRa.RX_CONTINUOUS)
        if done is False:
            print(f'[LORA] ⚠️ TX timeout sending "{line.strip()}"', flush=True)

radio = None
radio_init_lock = threading.Lock()
io_lock = threading.Lock()   # one request/response exchange at a time


def _want_sx1262():
    if LORA_BACKEND in ("sx1262", "spi"):
        return True
    if LORA_BACKEND == "serial":
        return False
    try:
        import LoRaRF  # noqa: F401
    except ImportError:
        return False
    return os.path.exists(f"/dev/spidev{SPI_BUS}.{SPI_CS}")


def ensure_radio():
    """Open the radio on first use (works with 'flask run' too) and after a failure."""
    global radio
    with radio_init_lock:
        if radio is not None and radio.alive:
            return True
        try:
            radio = Sx1262Radio() if _want_sx1262() else SerialRadio()
        except Exception as e:
            radio = None
            print(f"[!] Radio init failed (backend={LORA_BACKEND}): {e}", flush=True)
            print("[!] serial: right port? (ls /dev/ttyUSB* /dev/ttyACM*) user in 'dialout'?", flush=True)
            print("[!] sx1262: SPI enabled? rpi-lgpio installed? drone program / lora-c.py stopped?", flush=True)
            return False
        return True


def transmit(cmd):
    radio.send(cmd)
    print(f"[TX] {cmd}", flush=True)


def command_action(cmd):
    """'1-land' -> 'land', '1-t 20' -> 't'"""
    action = cmd.split('-', 1)[1] if '-' in cmd else cmd
    return action.strip().split(' ')[0].lower()


def reply_matches(cmd, line):
    """True if a received line is the answer to cmd. The drone (command_dispatcher.cpp)
    replies '[Drone <id>] <first word of command>: ...', status replies with bare fields
    ('[Drone 1] A:N|M:...'), and with --lora-ack LoraLink sends 'ACK <id> <command>'.
    Lines for other commands/drones are ignored here (still recorded as telemetry)."""
    drone_id = cmd.split('-', 1)[0]
    action = command_action(cmd)
    if line.startswith("ACK"):
        words = line[3:].split()
        if len(words) >= 2 and words[0] == drone_id:     # "ACK 1 t 20"
            return words[1].lower() == action
        return bool(words) and words[0].lower() == action  # older firmware: "ACK t 20"
    parsed = parse_drone_line(line)
    if parsed is None or parsed[0] != drone_id:
        return False
    keyword = (parsed[1] or "").lower()
    if not keyword:
        return action == "status"
    return keyword == action


def drain_rx():
    try:
        while True:
            radio.rx_queue.get_nowait()
    except queue.Empty:
        pass


_CMD_RE = re.compile(r'^[A-Za-z0-9_]+-[\x20-\x7e]{1,100}$')


def execute_command(cmd):
    """Send one '<id>-<command>' over LoRa and wait for the drone's reply.
    Returns (result_dict, http_status). Used by the web dashboard AND the tower socket."""
    if not ensure_radio():
        return {"error": "LoRa radio not connected - see Flask terminal"}, 500
    cmd = (cmd or "").strip()
    if not _CMD_RE.match(cmd):
        return {"error": f"bad command {cmd!r} - expected '<drone id>-<command>', e.g. 1-status"}, 400
    action = command_action(cmd)

    # "all-<cmd>": every drone runs it (LoraLink::handlePacket). The drones only accept
    # land / l / stop / estop / rtl this way and never ACK it, so send once, don't wait.
    if cmd.lower().startswith(ALL_PREFIX):
        if action not in BROADCAST_COMMANDS:
            return {"error": f"'{cmd}': drones ignore broadcast '{action}' - only "
                             f"{', '.join(sorted(BROADCAST_COMMANDS))} can go to all"}, 400
        try:
            transmit(cmd)
        except Exception as e:
            return {"error": str(e)}, 500
        return {"status": "success", "command": cmd,
                "response": "SENT to all drones - broadcasts are never acknowledged"}, 200

    # Emergency commands go out immediately, even while a poll is waiting for its reply.
    if action in EMERGENCY_COMMANDS and not io_lock.acquire(blocking=False):
        try:
            transmit(cmd)
        except Exception as e:
            return {"error": str(e)}, 500
        return {"status": "success", "command": cmd,
                "response": "SENT (priority) - reply not awaited"}, 200
    elif action not in EMERGENCY_COMMANDS:
        io_lock.acquire()
    # (an emergency command that got the lock without waiting continues here, lock held)

    try:
        drain_rx()                 # drop stale lines from earlier exchanges
        transmit(cmd)

        wants_telemetry = action in TELEMETRY_COMMANDS
        deadline = time.time() + MAX_WAIT_SECONDS
        ack, response = "", ""
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            try:
                line = radio.rx_queue.get(timeout=min(remaining, 0.1))
            except queue.Empty:
                continue
            if not reply_matches(cmd, line):
                continue          # a late reply to another command, or another drone
            if line.startswith("ACK"):
                ack = line
                if not wants_telemetry:
                    break
                continue
            response = line
            break

        if response:
            parsed = parse_drone_line(response)
            tel = None
            if parsed:
                with telemetry_lock:
                    st = drones.get(parsed[0])
                    tel = _with_online(st) if st else None
            return {"status": "success", "command": cmd, "response": response,
                    "ack": ack, "telemetry": tel}, 200
        if ack:
            return {"status": "success", "command": cmd, "response": ack, "ack": ack}, 200
        print(f"[!] No reply to '{cmd}' within {MAX_WAIT_SECONDS}s", flush=True)
        return {"status": "timeout", "command": cmd, "message": "No response from drone"}, 408

    except Exception as e:
        print(f"[!] Radio error: {e}", flush=True)
        if radio is not None:
            radio.alive = False     # re-open on the next request
        return {"error": str(e)}, 500
    finally:
        io_lock.release()


@app.route('/send', methods=['POST', 'GET'])
def send_command():
    if request.method == 'POST':
        data = request.get_json(silent=True)
        cmd = data.get('cmd') if data else request.form.get('cmd')
    else:
        cmd = request.args.get('cmd')

    if not cmd:
        return jsonify({"error": "No command provided. Use ?cmd=1-arm or send JSON {'cmd': '1-arm'}"}), 400

    result, status = execute_command(cmd)
    return jsonify(result), status


# ----------------------------------------------------------------------------
# Synaix WebSocket link  (one socket per drone this ground station relays)
#
# Each drone gets its own connection, registered as that drone:
#   wss://api.synaix.viclyx.com/ws?deviceType=drone&droneId=DRONE-1&mac=<mac>[&key=...]
# so the server's execute_command for DRONE-1 arrives here and is relayed over LoRa.
#
# Sent up the socket (WS_SEND=1, default):
#   {"type": "telemetry", "payload": {"lat", "lon", "altitude", "satellites", "status",
#                                     "armed", "mode", "speed", "verticalSpeed", "tracking",
#                                     "gpsFix", "gpsEnabled", "rssi", "snr", "gotoRemaining"}}
#       on every message from that drone (fields the drone didn't report are left out)
#   {"type": "command_ack", "cmdId", "droneId", "status": completed|failed|timeout, "detail"}
#   {"type": "ping"}  every 60 s keep-alive
# Received: {"type": "connected"} (logged), {"type": "execute_command", ...} (relayed),
#           broadcasts {"event": ...} (ignored). Plain text "1-status" also works.
#
# Server command -> drone command:
#   ARM arm | DISARM disarm | LAND land | STOP stop | ESTOP / EMERGENCY_STOP estop | RTL rtl
#   TAKEOFF {altitude} t <alt> | DESCEND {meters} d <m> | STATUS status | PING ping | SENSORS sensors
#   START_TRACKING / TRACK ys | STOP_TRACKING yz | TARE tare | SNAPSHOT snap | RESET reset
#   GOTO / GYS / TARGET {lat, lon, altitude?}  gys <lat> <lon> [alt]   (fly there, chase balloon)
#   SET_HOME {lat, lon, altitude?} sethome ... (no params: sethome here) | AREA {px} area <px>
#   SET_PARAM {name, value} set <name> <value>
#   lowercase text is passed straight through (e.g. "t 20"); unknown UPPERCASE commands are refused.
#
# Environment:
#   WS_URL=wss://...          default wss://api.synaix.viclyx.com/ws (https:// -> wss:// automatically)
#   WS_DRONES=1               drone numbers to connect as, e.g. "1,2,3"
#   WS_DRONE_ID=DRONE-{n}     droneId pattern
#   WS_MAC_1=aa:bb:...        MAC per drone (default 02:00:00:00:00:0<n>)
#   WS_KEY=...                optional &key=...
#   WS_SEND=0                 listen-only (no telemetry / acks / pings)
#   WS_POLL_S=3               if a drone has been quiet this long, ping it over LoRa so the
#                             server keeps getting data (0 = off; the dashboard polls too)
#   WS_TOWER=1                also connect as the tower (TOWER_ID, default Tower-1), listen-only
#   WS_ENABLE=0               no WebSocket at all
# ----------------------------------------------------------------------------
def _ws_url(url):
    """WebSockets use ws:// / wss://; accept http(s):// too and convert."""
    url = url.strip()
    if url.startswith("https://"):
        return "wss://" + url[len("https://"):]
    if url.startswith("http://"):
        return "ws://" + url[len("http://"):]
    return url


WS_URL = _ws_url(os.environ.get('WS_URL', os.environ.get('TOWER_WS_URL', 'wss://api.synaix.viclyx.com/ws')))
WS_DRONES = [d.strip() for d in os.environ.get('WS_DRONES', '1').split(',') if d.strip()]
WS_DRONE_ID = os.environ.get('WS_DRONE_ID', 'DRONE-{n}')
WS_KEY = os.environ.get('WS_KEY', os.environ.get('TOWER_KEY', '')).strip()
WS_SEND = os.environ.get('WS_SEND', '1') != '0'
WS_POLL_S = float(os.environ.get('WS_POLL_S', '3'))
WS_TOWER = os.environ.get('WS_TOWER', '0') == '1'
TOWER_ID = os.environ.get('TOWER_ID', 'Tower-1')
WS_ENABLE = os.environ.get('WS_ENABLE', os.environ.get('TOWER_LISTEN', '1')) != '0'

# ---- "target_locked" events from the tower camera ----
# Default (manual): the latest lock is stored and shown on the dashboard's GO & TRACK
# box (GET /target); nothing is sent to a drone until the operator clicks the button.
#   WS_TARGET_AUTO=1            send the go-to automatically instead (rate-limited, below)
#   WS_TARGET_DRONE=1           which drone is sent after the target (default: first of WS_DRONES)
#   WS_TARGET_POINT=estimated   use target.estimated_gps, or "predicted" for predicted_trajectory.predicted_gps
#   WS_TARGET_ALT_BELOW=5       fly this many metres BELOW the balloon (the camera looks up)
#   WS_TARGET_ALT_MIN=3 / WS_TARGET_ALT_MAX=60   clamp the go-to altitude
#   WS_TARGET_MIN_CONF=0.5      ignore locks with lower confidence
#   WS_TARGET_MIN_MOVE_M=10     only re-send when the target moved this far...
#   WS_TARGET_REFRESH_S=15      ...or this long passed since the last send
#   WS_TARGET_MIN_GAP_S=3       never send more often than this (LoRa airtime)
#   WS_TARGET_HOLDOFF_S=20      after the drone reports "BALLOON SEEN", stop sending go-tos
#                               for this long so the chase isn't interrupted
TARGET_AUTO = os.environ.get('WS_TARGET_AUTO', '0') == '1'
TARGET_DRONE = os.environ.get('WS_TARGET_DRONE', WS_DRONES[0] if WS_DRONES else '1')
TARGET_POINT = os.environ.get('WS_TARGET_POINT', 'estimated').lower()
TARGET_ALT_BELOW = float(os.environ.get('WS_TARGET_ALT_BELOW', '5'))
TARGET_ALT_MIN = float(os.environ.get('WS_TARGET_ALT_MIN', '3'))
TARGET_ALT_MAX = float(os.environ.get('WS_TARGET_ALT_MAX', '60'))
TARGET_MIN_CONF = float(os.environ.get('WS_TARGET_MIN_CONF', '0.5'))
TARGET_MIN_MOVE_M = float(os.environ.get('WS_TARGET_MIN_MOVE_M', '10'))
TARGET_REFRESH_S = float(os.environ.get('WS_TARGET_REFRESH_S', '15'))
TARGET_MIN_GAP_S = float(os.environ.get('WS_TARGET_MIN_GAP_S', '3'))
TARGET_HOLDOFF_S = float(os.environ.get('WS_TARGET_HOLDOFF_S', '20'))

_chasing_until = {}           # drone number -> time until which go-tos are suppressed
latest_target = None          # last target_locked, ready for the dashboard (GET /target)
_latest_target_seq = 0
_last_target_sent = {}        # drone number -> (time, lat, lon, alt)
_target_lock = threading.Lock()


def _offset_m(lat1, lon1, lat2, lon2):
    import math
    n = (lat2 - lat1) * 111320.0
    e = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
    return math.hypot(n, e)


def target_event_to_goto(msg):
    """Parse a target_locked event -> (lat, lon, alt, note) or (None, reason)."""
    body = msg.get("data") if isinstance(msg.get("data"), dict) and "target" in msg["data"] else msg
    t = body.get("target")
    if not isinstance(t, dict):
        return None, "no target object"
    if t.get("locked") is False:
        return None, "target not locked"
    conf = _num_or_none(t.get("confidence"))
    if conf is not None and conf < TARGET_MIN_CONF:
        return None, f"confidence {conf:.2f} < {TARGET_MIN_CONF}"

    point, which = None, "estimated_gps"
    if TARGET_POINT.startswith("pred"):
        pt = (t.get("predicted_trajectory") or {}).get("predicted_gps")
        if isinstance(pt, dict):
            point, which = pt, "predicted_gps"
    if point is None:
        point = t.get("estimated_gps") if isinstance(t.get("estimated_gps"), dict) else None
    if point is None:
        return None, "no estimated_gps"
    lat, lon = _num_or_none(point.get("lat")), _num_or_none(point.get("lon"))
    if lat is None or lon is None or abs(lat) > 90 or abs(lon) > 180 or (lat == 0 and lon == 0):
        return None, "bad target lat/lon"

    # Height above the ground at the tower ~= height above the drone's take-off point.
    tower = body.get("tower") if isinstance(body.get("tower"), dict) else {}
    tower_h = _num_or_none(tower.get("height_m")) or 0.0
    h = None
    if which == "predicted_gps" and _num_or_none(point.get("height_m")) is not None:
        h = _num_or_none(point.get("height_m")) + tower_h      # predicted height is relative to the camera
    if h is None:
        h = _num_or_none(t.get("height_above_ground_m"))
    if h is None:
        h = _num_or_none(point.get("height_m"))
    if h is None:
        h = _num_or_none(t.get("height_m"))
    if h is None:
        return None, "no target height"
    alt = min(max(h - TARGET_ALT_BELOW, TARGET_ALT_MIN), TARGET_ALT_MAX)
    note = (f"{which}, balloon {h:.1f} m AGL -> fly at {alt:.1f} m"
            + (f", conf {conf:.2f}" if conf is not None else "")
            + (f", {t.get('distance_m')} m from tower" if t.get('distance_m') is not None else ""))
    return (lat, lon, alt, note), None


def store_target(msg):
    """Keep the latest lock for the dashboard. Returns True if it was usable."""
    global latest_target, _latest_target_seq
    res, why = target_event_to_goto(msg)
    if res is None:
        print(f"[TARGET] ignored target_locked: {why}", flush=True)
        return False
    lat, lon, alt, note = res
    body = msg.get("data") if isinstance(msg.get("data"), dict) and "target" in msg["data"] else msg
    t = body.get("target") or {}
    with _target_lock:
        _latest_target_seq += 1
        latest_target = {
            "seq": _latest_target_seq,
            "received_at": _now_iso(),
            "event_time": body.get("timestamp", msg.get("timestamp")),
            "drone": TARGET_DRONE,
            "lat": round(lat, 7), "lon": round(lon, 7), "alt": round(alt, 1),
            "note": note,
            "confidence": t.get("confidence"),
            "distance_m": t.get("distance_m"),
            "compass": t.get("compass"),
            "height_agl_m": t.get("height_above_ground_m"),
            "lock_type": t.get("lock_type"),
            "command": f"{TARGET_DRONE}-gys {lat:.6f} {lon:.6f} {alt:.1f}",
            "tower": {"lat": _num_or_none((body.get("tower") or {}).get("lat")),
                      "lon": _num_or_none((body.get("tower") or {}).get("lon"))},
        }
    return True


# ----------------------------------------------------------------------------
# Go-to from "my location + distance + compass direction"
#   GET /calc-goto?lat=43.75861&lon=-79.42124&distance=120&bearing=SW&alt=20&drone=1
#   bearing: degrees (0 = north, 90 = east) or a compass point (N, NNE, NE, ... NNW)
#   -> {"lat", "lon", "alt", "bearing_deg", "distance_m", "command": "1-gys <lat> <lon> [alt]"}
# Nothing is sent: the dashboard fills GO & TRACK and the operator clicks to send.
# ----------------------------------------------------------------------------
COMPASS_POINTS = {name: i * 22.5 for i, name in enumerate(
    "N NNE NE ENE E ESE SE SSE S SSW SW WSW W WNW NW NNW".split())}
CALC_MAX_DISTANCE_M = 1000.0


def parse_bearing(value):
    """'SW' -> 225.0, 'n' -> 0.0, '217.5' -> 217.5; raises ValueError."""
    text = str(value or "").strip().upper().replace("°", "").replace("DEG", "").strip()
    if text in COMPASS_POINTS:
        return COMPASS_POINTS[text]
    try:
        deg = float(text)
    except ValueError:
        raise ValueError(f"bearing {value!r}: use degrees (0-360) or N, NE, SW, WNW ...")
    if not (deg == deg) or deg < 0 or deg > 360:
        raise ValueError("bearing must be 0-360 degrees")
    return deg % 360.0


def destination_point(lat, lon, distance_m, bearing_deg):
    """Point reached by going distance_m from (lat, lon) along bearing_deg (great circle)."""
    import math
    R = 6371008.8
    d = distance_m / R
    b = math.radians(bearing_deg)
    p1, l1 = math.radians(lat), math.radians(lon)
    p2 = math.asin(math.sin(p1) * math.cos(d) + math.cos(p1) * math.sin(d) * math.cos(b))
    l2 = l1 + math.atan2(math.sin(b) * math.sin(d) * math.cos(p1),
                         math.cos(d) - math.sin(p1) * math.sin(p2))
    return math.degrees(p2), (math.degrees(l2) + 540.0) % 360.0 - 180.0


# ----------------------------------------------------------------------------
# This computer's own GPS (for "MY GPS" on the dashboard)
#   default: /dev/ttyACM0    the laptop's USB GPS (found with find_gps.py; baud auto-detected)
#   MY_GPS=/dev/ttyUSB1      a different port
#   MY_GPS=gpsd              read from gpsd
#   MY_GPS=auto              scan serial ports (skips the LoRa module's port)
#   MY_GPS=off               don't read a GPS (use this where ttyACM0 is something else,
#                            e.g. a flight controller)
# Run find_gps.py first to see which port the GPS is on.
# ----------------------------------------------------------------------------
MY_GPS = os.environ.get('MY_GPS', '/dev/ttyACM0').strip()
my_gps = None


def start_my_gps():
    global my_gps
    if not MY_GPS or MY_GPS.lower() in ("0", "off", "no"):
        return
    try:
        import gps_reader
    except ImportError:
        print("[GPS] MY_GPS set but gps_reader.py is missing next to flask_lora.py", flush=True)
        return
    exclude = [SERIAL_PORT] if not _want_sx1262() else []
    if MY_GPS.startswith("/dev/") and not os.path.exists(MY_GPS):
        print(f"[GPS] {MY_GPS} not found - MY GPS button disabled (plug the GPS in, or set MY_GPS=...)",
              flush=True)
        return
    if MY_GPS.startswith("/dev/") and not _want_sx1262() and \
            os.path.realpath(MY_GPS) == os.path.realpath(SERIAL_PORT):
        print(f"[GPS] {MY_GPS} is the LoRa module's port - not reading GPS there", flush=True)
        return
    my_gps = gps_reader.GpsReader(MY_GPS, exclude=exclude).start()
    print(f"[GPS] reading this computer's GPS ({MY_GPS})", flush=True)


@app.route('/my-location')
def my_location():
    """This computer's GPS fix, if MY_GPS is configured."""
    if my_gps is None:
        return jsonify({"available": False,
                        "error": "no GPS configured - start Flask with MY_GPS=<port> (see find_gps.py)"})
    s = my_gps.snapshot()
    ok = bool(s["fix"] and s["lat"] is not None and (s["age_s"] is not None and s["age_s"] < 10))
    return jsonify({"available": ok, "lat": s["lat"], "lon": s["lon"], "alt": s["alt"],
                    "sats": s["sats"], "hdop": s["hdop"], "age_s": s["age_s"],
                    "device": s["device"],
                    "error": None if ok else (s["error"] or "GPS has no fix yet")})


@app.route('/calc-goto', methods=['GET', 'POST'])
def calc_goto():
    args = request.get_json(silent=True) if request.method == 'POST' else None
    args = args or request.values
    try:
        lat, lon = _num_or_none(args.get("lat")), _num_or_none(args.get("lon"))
        if lat is None or lon is None or abs(lat) > 90 or abs(lon) > 180 or (lat == 0 and lon == 0):
            raise ValueError("my location needs a valid lat and lon")
        dist = _num_or_none(args.get("distance"))
        if dist is None or dist <= 0 or dist > CALC_MAX_DISTANCE_M:
            raise ValueError(f"distance must be between 0 and {CALC_MAX_DISTANCE_M:g} m")
        bearing = parse_bearing(args.get("bearing"))
        alt = _num_or_none(args.get("alt")) if str(args.get("alt", "")).strip() else None
        drone = str(args.get("drone") or TARGET_DRONE).strip()
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    tlat, tlon = destination_point(lat, lon, dist, bearing)
    cmd = f"{drone}-gys {tlat:.6f} {tlon:.6f}" + (f" {alt:.1f}" if alt is not None else "")
    return jsonify({"lat": round(tlat, 7), "lon": round(tlon, 7), "alt": alt,
                    "bearing_deg": bearing, "distance_m": dist, "from": {"lat": lat, "lon": lon},
                    "command": cmd})


@app.route('/target')
def target_latest():
    """Latest tower target lock, for the dashboard to fill the GO & TRACK box."""
    with _target_lock:
        t = dict(latest_target) if latest_target else None
    if t is None:
        return jsonify({"target": None})
    age = (datetime.now(timezone.utc) - datetime.fromisoformat(t["received_at"])).total_seconds()
    t["age_s"] = round(age, 1)
    return jsonify({"target": t})


def target_job_for(drone_num, msg):
    """Rate-limited gys job for a target_locked event, or None."""
    res, why = target_event_to_goto(msg)
    if res is None:
        print(f"[TARGET] ignored target_locked: {why}", flush=True)
        return None
    lat, lon, alt, note = res
    now = time.time()
    with _target_lock:
        if now < _chasing_until.get(drone_num, 0):
            return None                       # drone's own camera has it: don't interrupt the chase
        last = _last_target_sent.get(drone_num)
        if last:
            t0, la0, lo0, al0 = last
            moved = max(_offset_m(la0, lo0, lat, lon), abs(alt - al0))
            if now - t0 < TARGET_MIN_GAP_S:
                return None
            if moved < TARGET_MIN_MOVE_M and now - t0 < TARGET_REFRESH_S:
                return None
        _last_target_sent[drone_num] = (now, lat, lon, alt)
    print(f"[TARGET] lock -> drone {drone_num}: {lat:.6f},{lon:.6f} @ {alt:.1f} m ({note})", flush=True)
    return {"cmd": f"{drone_num}-gys {lat:.6f} {lon:.6f} {alt:.1f}", "cmd_id": None,
            "server_drone": None, "kind": "target"}
WS_TOKEN = os.environ.get('TOWER_TOKEN', '').strip()     # optional Bearer header

def _short_reason(err):
    """One-line reason from a websocket-client error."""
    code = getattr(err, "status_code", None)
    if code:
        body = getattr(err, "resp_body", None)
        body = body.decode(errors="replace").strip() if isinstance(body, (bytes, bytearray)) else ""
        hint = {400: "server rejected the WebSocket handshake - check nginx Upgrade/Connection headers",
                401: "unauthorised - set TOWER_TOKEN",
                403: "forbidden - wrong TOWER_TOKEN or tower not allowed",
                404: "no WebSocket at this path",
                502: "backend down behind nginx",
                503: "backend unavailable"}.get(code, "")
        return f"HTTP {code}" + (f" {body[:60]}" if body else "") + (f" ({hint})" if hint else "")
    text = str(err).strip() or err.__class__.__name__
    return text.splitlines()[0][:160]


def _num_or_none(v):
    try:
        f = float(v)
        return f if f == f else None      # reject NaN
    except (TypeError, ValueError):
        return None


_SIMPLE_SERVER_CMDS = {
    "ARM": "arm", "DISARM": "disarm", "LAND": "land", "STOP": "stop",
    "ESTOP": "estop", "EMERGENCY_STOP": "estop", "E_STOP": "estop",
    "RTL": "rtl", "RETURN": "rtl", "RETURN_TO_LAUNCH": "rtl",
    "STATUS": "status", "PING": "ping", "SENSORS": "sensors", "TARE": "tare",
    "SNAPSHOT": "snap", "SNAP": "snap", "RESET": "reset",
    "START_TRACKING": "ys", "TRACK": "ys", "START": "ys", "YS": "ys",
    "STOP_TRACKING": "yz", "YZ": "yz",
}


def _drone_number(drone_id):
    """'DRONE-1' -> '1', 'drone_2' -> '2', '3' -> '3'; None if there is no number."""
    m = re.search(r'(\d+)\s*$', str(drone_id or ""))
    return m.group(1) if m else None


def _server_command_to_lora(command, params):
    """Translate one server command (+params) to the drone's text command, or raise ValueError."""
    params = params if isinstance(params, dict) else {}
    raw = str(command or "").strip()
    if not raw:
        raise ValueError("empty command")
    name = raw.upper().replace("-", "_").replace(" ", "_")

    def num(*keys):
        for k in keys:
            v = _num_or_none(params.get(k))
            if v is not None:
                return v
        return None

    if name in _SIMPLE_SERVER_CMDS:
        return _SIMPLE_SERVER_CMDS[name]
    if name == "TAKEOFF":
        alt = num("altitude", "alt", "alt_m", "height")
        if alt is None or alt <= 0:
            raise ValueError("TAKEOFF needs params.altitude > 0")
        return f"t {alt:g}"
    if name in ("DESCEND", "DOWN"):
        m = num("meters", "distance", "altitude", "alt")
        if m is None or m <= 0:
            raise ValueError("DESCEND needs params.meters > 0")
        return f"d {m:g}"
    if name in ("GOTO", "GO_TO", "GYS", "TARGET", "FLY_TO", "GOTO_TARGET"):
        tgt = params.get("target") if isinstance(params.get("target"), dict) else params
        lat = _num_or_none(tgt.get("lat", tgt.get("latitude")))
        lon = _num_or_none(tgt.get("lon", tgt.get("lng", tgt.get("longitude"))))
        alt = _num_or_none(tgt.get("altitude", tgt.get("alt", tgt.get("alt_m"))))
        if lat is None or lon is None or abs(lat) > 90 or abs(lon) > 180:
            raise ValueError(f"{name} needs params.lat and params.lon")
        return f"gys {lat:.6f} {lon:.6f}" + (f" {alt:.1f}" if alt is not None else "")
    if name == "SET_HOME":
        lat, lon, alt = num("lat", "latitude"), num("lon", "lng", "longitude"), num("altitude", "alt")
        if lat is None or lon is None:
            return "sethome here"
        return f"sethome {lat:.6f} {lon:.6f}" + (f" {alt:g}" if alt is not None else "")
    if name == "AREA":
        px = num("px", "area", "value")
        if px is None:
            raise ValueError("AREA needs params.px")
        return f"area {px:g}"
    if name in ("SET", "SET_PARAM"):
        pname, val = params.get("name", params.get("param")), _num_or_none(params.get("value"))
        if not pname or val is None:
            raise ValueError("SET_PARAM needs params.name and params.value")
        return f"set {pname} {val:g}"
    if raw != raw.upper():           # lowercase / mixed text: pass straight through ("t 20")
        return raw
    raise ValueError(f"unsupported command {raw!r}")


class DeviceSocket:
    """One WebSocket connection to the backend, registered as one device."""

    def __init__(self, device_type, device_id, drone_num=None, mac=None, send=True):
        self.device_type, self.device_id, self.drone_num = device_type, device_id, drone_num
        self.mac, self.send_enabled = mac, send
        self.tag = f"[WS {device_id}]"
        self.ws = None
        self.out = queue.Queue(maxsize=200)
        self.poll_busy = False

    # ---- url / sending ----
    def url(self, mask_key=False):
        if "deviceType=" in WS_URL:
            return WS_URL
        q = {"deviceType": self.device_type, "droneId": self.device_id}
        if self.mac:
            q["mac"] = self.mac
        if WS_KEY:
            q["key"] = "***" if mask_key else WS_KEY
        return WS_URL + ("&" if "?" in WS_URL else "?") + urllib.parse.urlencode(q)

    @property
    def connected(self):
        return self.ws is not None

    def send(self, obj):
        """Queue a message. Never blocks or raises; dropped while disconnected."""
        if not self.send_enabled or self.ws is None:
            return False
        data = json.dumps(obj)
        try:
            self.out.put_nowait(data)
        except queue.Full:
            try:
                self.out.get_nowait()
                self.out.put_nowait(data)
            except (queue.Empty, queue.Full):
                pass
        return True

    def _sender(self):
        while True:
            msg = self.out.get()
            ws = self.ws
            if ws is None:
                continue
            try:
                ws.send(msg)
            except Exception as e:
                print(f"{self.tag} send failed ({_short_reason(e)})", flush=True)

    def _keepalive(self):
        while True:
            time.sleep(60)
            self.send({"type": "ping"})

    # ---- receiving ----
    def _jobs_from(self, raw):
        """Parse one message into [{cmd, cmd_id, server_drone}] (raises ValueError on a bad command)."""
        try:
            msg = json.loads(raw)
        except (TypeError, ValueError):
            return [{"cmd": line.strip(), "cmd_id": None, "server_drone": None}
                    for line in str(raw).splitlines() if line.strip()]
        if isinstance(msg, list):
            out = []
            for m in msg:
                out += self._jobs_from(json.dumps(m))
            return out
        if not isinstance(msg, dict):
            return []

        mtype = str(msg.get("type", "")).lower()
        if mtype == "connected":
            print(f"{self.tag} server handshake: droneId={msg.get('droneId')} deviceType={msg.get('deviceType')} "
                  f"authenticated={msg.get('authenticated')} id={msg.get('connectionId')}", flush=True)
            return []
        if mtype == "error":
            print(f"{self.tag} server error: {msg.get('message') or msg.get('error') or msg}", flush=True)
            return []
        if str(msg.get("event", "")).lower() == "target_locked":
            if self.drone_num is None or self.drone_num != TARGET_DRONE:
                return []                    # only the designated drone's socket acts on it
            ok = store_target(msg)           # shown on the dashboard; operator clicks to send
            if ok and TARGET_AUTO:
                job = target_job_for(self.drone_num, msg)
                return [job] if job else []
            return []
        if "event" in msg or mtype not in ("execute_command", "command", ""):
            return []                    # broadcasts, pong, acks...: not for us

        cmd_id = msg.get("cmdId", msg.get("request_id", msg.get("id")))
        server_drone = msg.get("droneId", msg.get("drone_id"))
        if mtype in ("execute_command", "command"):
            if self.drone_num is None:
                raise ValueError(f"this socket is the tower ({self.device_id}); it relays nothing")
            num = _drone_number(server_drone) if server_drone else self.drone_num
            if num is None:
                raise ValueError(f"droneId {server_drone!r} has no drone number")
            lora = _server_command_to_lora(msg.get("command"), msg.get("params"))
            return [{"cmd": f"{num}-{lora}", "cmd_id": cmd_id,
                     "server_drone": server_drone or self.device_id}]
        cmd = msg.get("command", msg.get("cmd"))       # {"command": "1-status"}
        return [{"cmd": str(cmd).strip(), "cmd_id": cmd_id, "server_drone": server_drone}] if cmd else []

    def _run_job(self, job):
        cmd = job["cmd"]
        print(f"{self.tag} command from server: {cmd}" + (f"  (cmdId {job['cmd_id']})" if job["cmd_id"] else ""),
              flush=True)
        result, status = execute_command(cmd)
        detail = result.get('response') or result.get('message') or result.get('error')
        print(f"{self.tag} result: {cmd} -> {detail}", flush=True)
        ok = status == 200 and result.get("status") == "success"
        if job.get("kind") == "target":
            if detail and "REJECTED" in str(detail):
                print(f"{self.tag} drone refused the go-to: {detail}", flush=True)
            return
        self.send({"type": "command_ack", "cmdId": job["cmd_id"],
                   "droneId": job["server_drone"] or self.device_id,
                   "status": "completed" if ok else ("timeout" if status == 408 else "failed"),
                   "detail": detail})

    def _on_message(self, ws, raw):
        try:
            jobs = self._jobs_from(raw)
        except ValueError as e:
            print(f"{self.tag} refused command {str(raw)[:160]!r}: {e}", flush=True)
            try:
                m = json.loads(raw)
                self.send({"type": "command_ack", "cmdId": m.get("cmdId"),
                           "droneId": m.get("droneId", self.device_id), "status": "failed", "detail": str(e)})
            except Exception:
                pass
            return
        for job in jobs:
            # own thread per command: a 4 s wait never blocks a LAND behind it
            threading.Thread(target=self._run_job, args=(job,), daemon=True).start()

    # ---- connection loop ----
    def _loop(self, websocket):
        headers = [f"Authorization: Bearer {WS_TOKEN}"] if WS_TOKEN else []
        backoff, attempt = 2, 0
        print(f"{self.tag} connecting to {self.url(mask_key=True)} in the background", flush=True)
        while True:
            attempt += 1
            state = {"reason": None, "was_connected": False}

            def on_open(ws):
                nonlocal backoff, attempt
                self.ws = ws
                state["was_connected"] = True
                backoff, attempt = 2, 0
                mode = "sending telemetry + acks" if self.send_enabled else "listen-only"
                print(f"{self.tag} connected ({mode})", flush=True)
                if self.drone_num is not None:
                    _push_latest_to_socket(self)

            def on_close(ws, code, reason):
                self.ws = None
                if code or reason:
                    state["reason"] = state["reason"] or f"closed by server ({code} {reason})".strip()

            def on_error(ws, err):
                state["reason"] = _short_reason(err)

            try:
                app_ws = websocket.WebSocketApp(self.url(), header=headers, on_open=on_open,
                                                on_message=self._on_message, on_close=on_close,
                                                on_error=on_error)
                app_ws.run_forever(ping_interval=20, ping_timeout=10)
            except Exception as e:               # never let this thread die
                state["reason"] = _short_reason(e)
            self.ws = None

            why = state["reason"] or "connection closed"
            if state["was_connected"]:
                print(f"{self.tag} disconnected: {why} - reconnecting in {backoff}s", flush=True)
            else:
                print(f"{self.tag} can't connect (attempt {attempt}): {why} - retry in {backoff}s", flush=True)
            time.sleep(backoff)
            backoff = min(backoff * 2, 30)

    def start(self, websocket):
        threading.Thread(target=self._sender, daemon=True).start()
        if self.send_enabled:
            threading.Thread(target=self._keepalive, daemon=True).start()
        threading.Thread(target=self._loop, args=(websocket,), daemon=True).start()


drone_sockets = {}        # drone number ("1") -> DeviceSocket


def telemetry_payload(st):
    """Per-drone state -> the server's telemetry payload (unknown fields left out)."""
    gps = st.get("gps") or {}
    alt = st.get("altitude_m")
    armed = st.get("armed")
    status = None
    if armed is False:
        status = "idle"
    elif armed:
        status = "flying" if (alt is not None and alt > 0.5) else "armed"
    p = {
        "lat": gps.get("lat") if gps.get("fix") else None,
        "lon": gps.get("lon") if gps.get("fix") else None,
        "altitude": alt,
        "satellites": gps.get("satellites"),
        "status": status,
        "armed": armed,
        "mode": st.get("mode"),
        "speed": st.get("ground_speed_ms"),
        "verticalSpeed": st.get("climb_rate_ms"),
        "tracking": st.get("tracking"),
        "gpsEnabled": gps.get("enabled"),
        "gpsFix": gps.get("fix"),
        "rssi": (st.get("link") or {}).get("rssi_dbm"),
        "snr": (st.get("link") or {}).get("snr_db"),
        "gotoRemaining": (st.get("goto") or {}).get("remaining_m"),
    }
    return {k: v for k, v in p.items() if v is not None}


def _publish_telemetry(snapshot):
    sock = drone_sockets.get(str(snapshot.get("drone_id")))
    if sock is not None and sock.connected:
        payload = telemetry_payload(snapshot)
        if set(payload) - {"rssi", "snr"}:      # skip replies that carried no flight data
            sock.send({"type": "telemetry", "payload": payload})


def _push_latest_to_socket(sock):
    with telemetry_lock:
        st = drones.get(sock.drone_num)
        snap = json.loads(json.dumps(st)) if st else None
    if snap:
        _publish_telemetry(snap)


def _auto_poller():
    """Ping a connected drone over LoRa when it has been quiet for WS_POLL_S."""
    while True:
        time.sleep(1.0)
        for num, sock in list(drone_sockets.items()):
            if not sock.connected or sock.poll_busy:
                continue
            with telemetry_lock:
                st = drones.get(num)
                last = st["updated_at"] if st else None
            age = ((datetime.now(timezone.utc) - datetime.fromisoformat(last)).total_seconds()
                   if last else 1e9)
            if age < WS_POLL_S:
                continue

            def poll(n=num, s=sock):
                s.poll_busy = True
                try:
                    execute_command(f"{n}-ping")
                finally:
                    s.poll_busy = False
            threading.Thread(target=poll, daemon=True).start()


def start_ws_links():
    if not WS_ENABLE:
        return
    try:
        import websocket   # pip install websocket-client
    except ImportError:
        print("[WS] disabled: run  pip install websocket-client --break-system-packages", flush=True)
        return
    websocket.setdefaulttimeout(10)      # a dead network can't hang a connect attempt
    for n in WS_DRONES:
        mac = os.environ.get(f"WS_MAC_{n}") or f"02:00:00:00:00:{int(n) % 256:02x}"
        sock = DeviceSocket("drone", WS_DRONE_ID.format(n=n), drone_num=n, mac=mac, send=WS_SEND)
        drone_sockets[n] = sock
        sock.start(websocket)
    if WS_TOWER:
        DeviceSocket("tower", TOWER_ID, send=False).start(websocket)
    if WS_SEND and WS_POLL_S > 0 and drone_sockets:
        threading.Thread(target=_auto_poller, daemon=True).start()


start_tower_listener = start_ws_links     # old name



# ----------------------------------------------------------------------------
# GPS test flight:  take off -> hold altitude + position -> RTL -> landed
#   POST /test-flight/start  {"drone": "1", "alt": 5, "hold_s": 20, "max_drift_m": 8}
#   POST /test-flight/abort  -> LAND now
#   GET  /test-flight        -> status, step log, drift/altitude results
# Uses only commands the drone already has (status, t, ping, rtl, land).
# Any check that fails during the flight -> LAND. The operator can abort any time.
# ----------------------------------------------------------------------------
class TestFlight:
    def __init__(self):
        self.lock = threading.Lock()
        self.thread = None
        self.abort_flag = threading.Event()
        self.state = {"running": False, "phase": "idle", "result": None, "log": [], "metrics": {}}

    # ---- helpers ----
    def _log(self, msg):
        line = f"{datetime.now().strftime('%H:%M:%S')} {msg}"
        print(f"[TEST] {msg}", flush=True)
        with self.lock:
            self.state["log"].append(line)
            self.state["log"] = self.state["log"][-80:]

    def _phase(self, p):
        with self.lock:
            self.state["phase"] = p

    def status(self):
        with self.lock:
            return json.loads(json.dumps(self.state))

    def _tel(self, n):
        """Ping the drone and return its latest state (or None if it didn't answer)."""
        res, code = execute_command(f"{n}-ping")
        if code != 200:
            return None
        with telemetry_lock:
            st = drones.get(n)
            return json.loads(json.dumps(st)) if st else None

    def _sleep(self, s):
        return self.abort_flag.wait(s)          # True = abort requested

    def _land(self, n, why):
        self._log(f"ABORT: {why} -> LAND")
        self._phase("landing (abort)")
        execute_command(f"{n}-land")

    # ---- control ----
    def start(self, n, alt, hold_s, max_drift):
        with self.lock:
            if self.state["running"]:
                return False, "a test is already running"
            self.state = {"running": True, "phase": "preflight", "result": None, "log": [],
                          "metrics": {}, "drone": n, "alt": alt, "hold_s": hold_s, "max_drift_m": max_drift}
        self.abort_flag.clear()
        self.thread = threading.Thread(target=self._run, args=(n, alt, hold_s, max_drift), daemon=True)
        self.thread.start()
        return True, "started"

    def abort(self):
        n = self.state.get("drone") or "1"
        self.abort_flag.set()
        self._log("operator ABORT")
        execute_command(f"{n}-land")            # immediately, whatever phase we're in
        return True

    # ---- the sequence ----
    def _run(self, n, alt, hold_s, max_drift):
        result, why = "FAIL", ""
        airborne = False
        try:
            # 1. PREFLIGHT
            self._log(f"drone {n}: preflight checks (target {alt:g} m, hold {hold_s:g} s, max drift {max_drift:g} m)")
            execute_command(f"{n}-status")
            st = self._tel(n)
            if not st:
                why = "no reply from drone"; self._log("FAIL: " + why); return
            gps = st.get("gps") or {}
            problems = []
            if gps.get("enabled") is False:
                problems.append("drone is in --no-gps mode (GPS:OFF)")
            if not gps.get("fix"):
                problems.append("no GPS fix")
            if (gps.get("satellites") or 0) < 6:
                problems.append(f"only {gps.get('satellites') or 0} satellites (need 6+)")
            if st.get("armed"):
                problems.append("already armed")
            a0 = st.get("altitude_m")
            if a0 is None or abs(a0) > 1.5:
                problems.append(f"altitude reads {a0} m on the ground (should be ~0; baro/EKF offset)")
            if problems:
                why = "; ".join(problems); self._log("FAIL preflight: " + why); return
            home = (gps["lat"], gps["lon"])
            self._log(f"preflight OK: {gps.get('satellites')} sats, pos {home[0]:.6f},{home[1]:.6f}, alt {a0} m")

            # 2. TAKEOFF
            self._phase("takeoff")
            res, code = execute_command(f"{n}-t {alt:g}")
            reply = str(res.get("response") or res.get("message") or res.get("error"))
            self._log(f"takeoff command -> {reply}")
            if code != 200 or "OK" not in reply:
                why = f"takeoff refused: {reply}"; return
            airborne = True
            t0, reached = time.time(), False
            while time.time() - t0 < 30:
                if self._sleep(1.0):
                    why = "aborted by operator"; return
                st = self._tel(n)
                if not st:
                    continue
                a = st.get("altitude_m")
                self._log(f"  climbing: alt {a} m, mode {st.get('mode')}")
                if a is not None and a >= alt * 0.8:      # code stops native takeoff at ~85%
                    reached = True
                    break
                if a is not None and a > alt + 3:
                    self._land(n, f"overshoot to {a} m"); why = "overshoot"; return
            if not reached:
                self._land(n, "did not reach altitude in 30 s"); why = "takeoff timeout"; return

            # 3. HOLD
            self._phase("hold")
            if self._sleep(2.0):                                  # let it settle
                why = "aborted by operator"; return
            st = self._tel(n) or {}
            g = st.get("gps") or {}
            if not g.get("fix"):
                self._land(n, "lost GPS fix before hold"); why = "GPS lost"; return
            ref = (g["lat"], g["lon"])
            ref_alt = st.get("altitude_m")
            self._log(f"HOLD start at {ref[0]:.6f},{ref[1]:.6f}, alt {ref_alt} m - holding {hold_s:g} s")
            drifts, alts = [], []
            t0 = time.time()
            while time.time() - t0 < hold_s:
                if self._sleep(1.5):
                    why = "aborted by operator"; return
                st = self._tel(n)
                if not st:
                    self._log("  (no reply this time)")
                    continue
                g = st.get("gps") or {}
                a = st.get("altitude_m")
                if not g.get("fix"):
                    self._land(n, "GPS fix lost during hold"); why = "GPS lost"; return
                d = _offset_m(ref[0], ref[1], g["lat"], g["lon"])
                drifts.append(d)
                if a is not None:
                    alts.append(a)
                self._log(f"  hold t+{time.time()-t0:4.1f}s: drift {d:4.1f} m, alt {a} m, "
                          f"speed {st.get('ground_speed_ms')} m/s")
                if d > max_drift:
                    self._land(n, f"drifted {d:.1f} m (> {max_drift:g} m)"); why = "drift limit"; return
                if a is not None and abs(a - alt) > 2.5:
                    self._land(n, f"altitude {a} m out of {alt-2.5:g}-{alt+2.5:g} m"); why = "altitude limit"; return
            m = {"max_drift_m": round(max(drifts), 2) if drifts else None,
                 "avg_drift_m": round(sum(drifts) / len(drifts), 2) if drifts else None,
                 "alt_min_m": min(alts) if alts else None, "alt_max_m": max(alts) if alts else None,
                 "samples": len(drifts)}
            with self.lock:
                self.state["metrics"] = m
            self._log(f"HOLD done: max drift {m['max_drift_m']} m, avg {m['avg_drift_m']} m, "
                      f"alt {m['alt_min_m']}-{m['alt_max_m']} m")

            # 4. RTL
            self._phase("rtl")
            res, code = execute_command(f"{n}-rtl")
            reply = str(res.get("response") or res.get("message") or res.get("error"))
            self._log(f"RTL command -> {reply}")
            if code != 200 or "OK" not in reply:
                self._land(n, f"RTL not accepted ({reply})"); why = "RTL refused"; return
            t0 = time.time()
            while time.time() - t0 < 90:
                if self._sleep(2.0):
                    why = "aborted by operator"; return
                st = self._tel(n)
                if not st:
                    continue
                a = st.get("altitude_m")
                g = st.get("gps") or {}
                dh = _offset_m(home[0], home[1], g["lat"], g["lon"]) if g.get("fix") else None
                self._log(f"  returning: alt {a} m, {'%.1f' % dh if dh is not None else '?'} m from home, "
                          f"armed {st.get('armed')}")
                if st.get("armed") is False:
                    airborne = False
                    with self.lock:
                        self.state["metrics"]["landed_from_home_m"] = round(dh, 2) if dh is not None else None
                    result, why = "PASS", "landed and disarmed"
                    self._log(f"LANDED {('%.1f m from take-off point' % dh) if dh is not None else ''} - PASS")
                    return
            why = "RTL still in progress after 90 s - watch the drone"
            self._log(why)
        except Exception as e:
            why = f"test error: {e}"
            self._log(why)
            if airborne:
                self._land(n, "test error")
        finally:
            with self.lock:
                self.state["running"] = False
                self.state["result"] = {"status": result, "reason": why}
                self.state["phase"] = "done"
            print(f"[TEST] RESULT {result}: {why}", flush=True)


test_flight = TestFlight()


@app.route('/test-flight', methods=['GET'])
def test_flight_status():
    return jsonify(test_flight.status())


@app.route('/test-flight/start', methods=['POST'])
def test_flight_start():
    a = request.get_json(silent=True) or request.values
    n = str(a.get("drone") or "1").strip()
    alt = _num_or_none(a.get("alt")) or 5.0
    hold = _num_or_none(a.get("hold_s")) or 20.0
    drift = _num_or_none(a.get("max_drift_m")) or 8.0
    if not (2.0 <= alt <= 10.0):
        return jsonify({"error": "test altitude must be 2-10 m"}), 400
    if not (5.0 <= hold <= 120.0):
        return jsonify({"error": "hold time must be 5-120 s"}), 400
    if not (2.0 <= drift <= 30.0):
        return jsonify({"error": "max drift must be 2-30 m"}), 400
    ok, msg = test_flight.start(n, alt, hold, drift)
    return jsonify({"ok": ok, "message": msg}), (200 if ok else 409)


@app.route('/test-flight/abort', methods=['POST'])
def test_flight_abort():
    test_flight.abort()
    return jsonify({"ok": True, "message": "LAND sent"})

if __name__ == '__main__':
    ensure_radio()
    start_my_gps()
    start_ws_links()
    # threaded=True so an emergency LAND request isn't stuck behind a poll request.
    app.run(host='0.0.0.0', port=5000, threaded=True)

"""
Flask LoRa ground-station relay for the pilora dashboard.

Two radio backends:
  sx1262 - SX1262 wired to a Raspberry Pi over SPI (same pins / settings as
           lora-c.py, the setup confirmed talking to the drone both ways)
  serial - a USB/UART LoRa module on /dev/ttyUSB0 (transparent bridge)

LORA_BACKEND=auto (default) picks sx1262 when LoRaRF is installed and SPI is
enabled, otherwise serial. Override with environment variables, e.g.
  LORA_BACKEND=sx1262 python flask_lora.py
  LORA_BACKEND=serial LORA_PORT=/dev/ttyACM0 python flask_lora.py

Only ONE program can own the radio: stop lora-c.py / lora_serial.py /
ground_station.py before starting this.
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

# sx1262 backend - identical to lora-c.py ("confirmed working values")
SPI_BUS, SPI_CS = 0, 0
PIN_RESET, PIN_BUSY, PIN_IRQ, PIN_TXEN, PIN_RXEN = 22, 23, 24, 5, 6
FREQUENCY = 915_000_000
TX_POWER = 22
SPREADING_FACTOR = 9
BANDWIDTH = 125_000
CODING_RATE = 5
PREAMBLE_LEN = 12
SYNC_WORD = 0x12
REASSEMBLY_GAP_SECONDS = 0.2   # fragments closer than this are one message

# How long to wait for the drone's reply (ACK + telemetry can be two packets).
MAX_WAIT_SECONDS = 4.0

# Commands that must never queue behind a background poll.
EMERGENCY_COMMANDS = {"land", "stop", "estop"}

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
        if keyword == "gys" and text and not text.startswith("OK"):
            st["goto"] = {"active": False, "remaining_m": None}   # ARRIVED / BALLOON SEEN / TIMEOUT
        snapshot = json.loads(json.dumps(st))
    _queue_push(snapshot)
    tower_send({"type": "telemetry", "tower_id": TOWER_ID, "drone": _with_online(snapshot)})


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
# Radio backends. Both push every complete received line into rx_queue.
# ----------------------------------------------------------------------------
class SerialRadio:
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
                print(f"[RX] {line}", flush=True)
                record_line(line)
                self.rx_queue.put(line)

    def send(self, line):
        with self._tx_lock:
            self.ser.write(f"{line}\n".encode('utf-8'))
            self.ser.flush()


class Sx1262Radio:
    name = "sx1262"

    def __init__(self):
        from LoRaRF import SX126x
        self.rx_queue = queue.Queue()
        self._lock = threading.Lock()       # one SPI user at a time
        LoRa = SX126x()
        if not LoRa.begin(SPI_BUS, SPI_CS, PIN_RESET, PIN_BUSY, PIN_IRQ, PIN_TXEN, PIN_RXEN):
            raise RuntimeError("SX1262 init failed - check wiring / SPI enabled / "
                               "lora-c.py or another program not holding the pins")
        LoRa.setFrequency(FREQUENCY)
        LoRa.setTxPower(TX_POWER, LoRa.TX_POWER_SX1262)
        LoRa.setLoRaModulation(SPREADING_FACTOR, BANDWIDTH, CODING_RATE)
        LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, PREAMBLE_LEN, 15, crcType=True)
        LoRa.setSyncWord(SYNC_WORD)
        self.LoRa = LoRa
        self.alive = True
        with self._lock:
            LoRa.request(LoRa.RX_CONTINUOUS)
        threading.Thread(target=self._rx_loop, daemon=True).start()
        print(f"[*] SX1262 ready | {FREQUENCY / 1e6} MHz | SF{SPREADING_FACTOR} | {TX_POWER} dBm")

    def _push(self, parts, rssi, snr):
        text = "".join(parts)
        for line in text.replace("\r", "\n").split("\n"):
            line = line.strip()
            if line:
                print(f"[RX] {line}  (RSSI {rssi} dBm, SNR {snr} dB)", flush=True)
                record_line(line, rssi, snr)
                self.rx_queue.put(line)

    def _rx_loop(self):
        LoRa = self.LoRa
        parts, last_t, rssi, snr = [], None, None, None
        while self.alive:
            try:
                with self._lock:
                    if LoRa.status() == 7:                 # STATUS_RX_DONE
                        length = LoRa.available()
                        if length:
                            frag = bytes(LoRa.read(length)).decode(errors="replace")
                            now = time.time()
                            if parts and last_t is not None and now - last_t > REASSEMBLY_GAP_SECONDS:
                                self._push(parts, rssi, snr)
                                parts = []
                            rssi, snr = LoRa.packetRssi(), LoRa.snr()
                            parts.append(frag)
                            last_t = now
                        LoRa.request(LoRa.RX_CONTINUOUS)
            except Exception as e:
                print(f"[!] SX1262 RX error: {e}", flush=True)
                time.sleep(0.5)

            if parts and last_t is not None and time.time() - last_t > REASSEMBLY_GAP_SECONDS:
                self._push(parts, rssi, snr)
                parts, last_t = [], None
            time.sleep(0.02)

    def send(self, line):
        data = list(f"{line}\n".encode())
        with self._lock:
            LoRa = self.LoRa
            LoRa.beginPacket()
            LoRa.write(data, len(data))
            LoRa.endPacket()
            LoRa.wait()
            LoRa.request(LoRa.RX_CONTINUOUS)


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
            print("[!] sx1262: SPI enabled? rpi-lgpio installed? lora-c.py stopped?", flush=True)
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
    """True if a received line is the answer to cmd. The drone replies
    '[Drone <id>] <first word of command>: ...', status replies with bare
    fields ('[Drone 1] A:N|M:...'), and ACKs are 'ACK <command>'.
    Lines for other commands/drones are ignored here (still recorded as telemetry)."""
    drone_id = cmd.split('-', 1)[0]
    action = command_action(cmd)
    if line.startswith("ACK"):
        rest = line[3:].strip()
        return rest.split(' ')[0].lower() == action if rest else True
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
# Tower socket listener
#
# Keeps a WebSocket open to the Synaix backend. The backend sends commands
# (e.g. "1-status") and target GPS points; each command is relayed over LoRa
# and the result is sent back on the same socket. Every telemetry update is
# also streamed up the socket.
#
#   TOWER_ID=TWR-01            (default)
#   TOWER_WS_URL=wss://...     (default: wss://api.synaix.viclyx.com/api/towers/<TOWER_ID>/telemetry)
#   TOWER_TOKEN=secret         (optional, sent as "Authorization: Bearer secret")
#   TOWER_LISTEN=0             (disable the listener)
#
# Accepted incoming messages (text or JSON):
#   1-status                                         plain text, one command per line
#   {"command": "1-status"}                          also "cmd"
#   {"drone_id": "1", "command": "status"}           id prefixed automatically
#   {"type": "target", "drone_id": "1", "lat": 43.75861, "lon": -79.42124, "alt": 20}
#   {"drone_id": "1", "target": {"lat": ..., "lon": ..., "alt": ...}}
#       -> sent to the drone as "1-gys <lat> <lon> [alt]" (fly there, chase balloon)
#   Any JSON message may carry "request_id" (or "id"); it is echoed in the reply.
#
# Outgoing messages:
#   {"type": "hello", "tower_id": "TWR-01"}                          on connect
#   {"type": "command_result", "tower_id", "request_id", "command", "status", "response", "telemetry", ...}
#   {"type": "telemetry", "tower_id", "drone": {<per-drone JSON, same as /telemetry/<id>>}}
#   {"type": "error", "tower_id", "request_id", "error"}
# ----------------------------------------------------------------------------
TOWER_ID = os.environ.get('TOWER_ID', 'TWR-01')
TOWER_WS_URL = os.environ.get(
    'TOWER_WS_URL', f'wss://api.synaix.viclyx.com/api/towers/{TOWER_ID}/telemetry')
TOWER_TOKEN = os.environ.get('TOWER_TOKEN', '').strip()
TOWER_LISTEN = os.environ.get('TOWER_LISTEN', '1') != '0'

_tower_ws = None
_tower_out = queue.Queue(maxsize=200)   # outgoing messages; the sender thread drains it


def tower_send(obj):
    """Queue a JSON message for the tower socket. Never blocks and never raises:
    while the socket is down, messages are simply dropped (telemetry is sent
    again on the next update anyway), so the radio and dashboard never wait on it."""
    if _tower_ws is None:
        return False
    try:
        _tower_out.put_nowait(json.dumps(obj))
    except queue.Full:
        try:
            _tower_out.get_nowait()          # drop the oldest, keep the newest
            _tower_out.put_nowait(json.dumps(obj))
        except (queue.Empty, queue.Full):
            pass
    return True


def _tower_sender():
    while True:
        msg = _tower_out.get()
        ws = _tower_ws
        if ws is None:
            continue
        try:
            ws.send(msg)
        except Exception as e:
            print(f"[TOWER] send failed ({_short_reason(e)}) - will resend after reconnect", flush=True)


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


def tower_message_to_commands(raw):
    """Turn one socket message into a list of (command, request_id) or raise ValueError."""
    try:
        msg = json.loads(raw)
    except (TypeError, ValueError):
        # plain text: one command per line
        return [(line.strip(), None) for line in str(raw).splitlines() if line.strip()]

    if isinstance(msg, str):
        return [(msg.strip(), None)] if msg.strip() else []
    if isinstance(msg, list):
        out = []
        for m in msg:
            out += tower_message_to_commands(json.dumps(m))
        return out
    if not isinstance(msg, dict):
        raise ValueError("unsupported message")

    mtype = str(msg.get("type", "")).lower()
    if mtype in ("ping", "pong", "heartbeat", "hello", "ack", "welcome", "connected"):
        return []
    req_id = msg.get("request_id", msg.get("id"))
    drone = msg.get("drone_id", msg.get("drone", msg.get("droneId")))
    drone = str(drone).strip() if drone is not None else None

    # ---- target GPS -> gys ----
    tgt = msg.get("target") if isinstance(msg.get("target"), dict) else None
    if tgt is None and (mtype in ("target", "gps", "goto", "gys", "target_gps")
                        or ("lat" in msg and ("lon" in msg or "lng" in msg))):
        tgt = msg
    if tgt is not None and "command" not in msg and "cmd" not in msg:
        lat = _num_or_none(tgt.get("lat", tgt.get("latitude")))
        lon = _num_or_none(tgt.get("lon", tgt.get("lng", tgt.get("longitude"))))
        alt = _num_or_none(tgt.get("alt", tgt.get("altitude", tgt.get("alt_m"))))
        if lat is None or lon is None or abs(lat) > 90 or abs(lon) > 180:
            raise ValueError("target needs valid lat and lon")
        if not drone:
            raise ValueError("target needs drone_id")
        cmd = f"{drone}-gys {lat:.6f} {lon:.6f}" + (f" {alt:.1f}" if alt is not None else "")
        return [(cmd, req_id)]

    # ---- plain command ----
    cmd = msg.get("command", msg.get("cmd"))
    if not cmd:
        raise ValueError("no command or target in message")
    cmd = str(cmd).strip()
    if drone and not cmd.startswith(f"{drone}-"):
        cmd = f"{drone}-{cmd}"
    return [(cmd, req_id)]


def _run_tower_command(cmd, req_id):
    print(f"[TOWER] command from server: {cmd}", flush=True)
    result, status = execute_command(cmd)
    reply = {"type": "command_result", "tower_id": TOWER_ID, "request_id": req_id,
             "command": cmd, "http_status": status}
    reply.update(result)
    tower_send(reply)


def _on_tower_message(ws, raw):
    try:
        cmds = tower_message_to_commands(raw)
    except ValueError as e:
        print(f"[TOWER] ignored message {str(raw)[:120]!r}: {e}", flush=True)
        req_id = None
        try:
            req_id = json.loads(raw).get("request_id")
        except Exception:
            pass
        tower_send({"type": "error", "tower_id": TOWER_ID, "request_id": req_id, "error": str(e)})
        return
    for cmd, req_id in cmds:
        # own thread per command: a 4 s wait for one drone never blocks a LAND behind it
        threading.Thread(target=_run_tower_command, args=(cmd, req_id), daemon=True).start()


def _tower_loop():
    global _tower_ws
    try:
        import websocket   # pip install websocket-client
    except ImportError:
        print("[TOWER] listener disabled: run  pip install websocket-client --break-system-packages",
              flush=True)
        return
    websocket.setdefaulttimeout(10)      # a dead network can't hang a connect attempt
    headers = [f"Authorization: Bearer {TOWER_TOKEN}"] if TOWER_TOKEN else []
    threading.Thread(target=_tower_sender, daemon=True).start()
    backoff = 2
    attempt = 0
    print(f"[TOWER] connecting to {TOWER_WS_URL} in the background (dashboard/radio don't wait for it)",
          flush=True)
    while True:
        attempt += 1
        state = {"reason": None, "was_connected": False}

        def on_open(ws):
            global _tower_ws
            nonlocal backoff, attempt
            _tower_ws = ws
            state["was_connected"] = True
            backoff, attempt = 2, 0
            print(f"[TOWER] connected to {TOWER_WS_URL}", flush=True)
            tower_send({"type": "hello", "tower_id": TOWER_ID})

        def on_close(ws, code, reason):
            global _tower_ws
            _tower_ws = None
            if code or reason:
                state["reason"] = state["reason"] or f"closed by server ({code} {reason})".strip()

        def on_error(ws, err):
            state["reason"] = _short_reason(err)

        try:
            app_ws = websocket.WebSocketApp(TOWER_WS_URL, header=headers, on_open=on_open,
                                            on_message=_on_tower_message, on_close=on_close,
                                            on_error=on_error)
            app_ws.run_forever(ping_interval=20, ping_timeout=10)
        except Exception as e:                      # never let the listener thread die
            state["reason"] = _short_reason(e)
        _tower_ws = None

        why = state["reason"] or "connection closed"
        if state["was_connected"]:
            print(f"[TOWER] disconnected: {why} - reconnecting in {backoff}s", flush=True)
        else:
            print(f"[TOWER] can't connect (attempt {attempt}): {why} - retry in {backoff}s", flush=True)
        time.sleep(backoff)
        backoff = min(backoff * 2, 30)


def start_tower_listener():
    if TOWER_LISTEN:
        threading.Thread(target=_tower_loop, daemon=True).start()


if __name__ == '__main__':
    ensure_radio()
    start_tower_listener()
    # threaded=True so an emergency LAND request isn't stuck behind a poll request.
    app.run(host='0.0.0.0', port=5000, threaded=True)

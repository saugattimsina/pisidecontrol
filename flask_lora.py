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
import os
import queue
import threading
import time

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


def drain_rx():
    try:
        while True:
            radio.rx_queue.get_nowait()
    except queue.Empty:
        pass


@app.route('/send', methods=['POST', 'GET'])
def send_command():
    if not ensure_radio():
        return jsonify({"error": "LoRa radio not connected - see Flask terminal"}), 500

    if request.method == 'POST':
        data = request.get_json(silent=True)
        cmd = data.get('cmd') if data else request.form.get('cmd')
    else:
        cmd = request.args.get('cmd')

    if not cmd:
        return jsonify({"error": "No command provided. Use ?cmd=1-arm or send JSON {'cmd': '1-arm'}"}), 400

    cmd = cmd.strip()
    action = command_action(cmd)

    # Emergency commands go out immediately, even while a poll is waiting for its reply.
    if action in EMERGENCY_COMMANDS and not io_lock.acquire(blocking=False):
        try:
            transmit(cmd)
        except Exception as e:
            return jsonify({"error": str(e)}), 500
        return jsonify({"status": "success", "command": cmd,
                        "response": "SENT (priority) - reply not awaited"}), 200
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
            if line.startswith("ACK"):
                ack = line
                if not wants_telemetry:
                    break
                continue
            response = line
            break

        if response:
            return jsonify({"status": "success", "command": cmd, "response": response, "ack": ack}), 200
        if ack:
            return jsonify({"status": "success", "command": cmd, "response": ack, "ack": ack}), 200
        print(f"[!] No reply to '{cmd}' within {MAX_WAIT_SECONDS}s", flush=True)
        return jsonify({"status": "timeout", "command": cmd, "message": "No response from drone"}), 408

    except Exception as e:
        print(f"[!] Radio error: {e}", flush=True)
        if radio is not None:
            radio.alive = False     # re-open on the next request
        return jsonify({"error": str(e)}), 500
    finally:
        io_lock.release()


if __name__ == '__main__':
    ensure_radio()
    # threaded=True so an emergency LAND request isn't stuck behind a poll request.
    app.run(host='0.0.0.0', port=5000, threaded=True)

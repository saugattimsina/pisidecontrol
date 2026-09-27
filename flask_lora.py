#!/usr/bin/env python3
"""
LoRa ground station web relay.

Browser  ->  Flask (/send)  ->  SX1262 on this Pi's SPI pins  ~~915 MHz~~>  drone

Default radio: an SX1262 wired to the ground Pi's SPI + GPIO pins, driven with
LoRaRF exactly like the known-good lora-c.py (same pins, same radio settings,
continuous RX, fragment reassembly, "\n" on every message).

Optional: --serial [/dev/ttyXXX] for a USB / UART LoRa module instead.

Run:  python3 flask_lora.py                  (SX1262 on SPI - default)
      python3 flask_lora.py --serial /dev/ttyUSB0 --baud 9600
Then open http://<this-pi-ip>:5000
"""

import argparse
import glob
import os
import queue
import threading
import time

from flask import Flask, request, jsonify, render_template

app = Flask(__name__)


@app.route('/')
def index():
    return render_template('index.html')


# ============================================================
# Radio settings - identical to lora-c.py (confirmed working)
# ============================================================
SPI_BUS   = 0
SPI_CS    = 0
PIN_RESET = 22
PIN_BUSY  = 23
PIN_IRQ   = 24   # DIO1
PIN_TXEN  = 5
PIN_RXEN  = 6

FREQUENCY        = 915_000_000
TX_POWER         = 22
SPREADING_FACTOR = 9
BANDWIDTH        = 125_000
CODING_RATE      = 5
PREAMBLE_LEN     = 12
SYNC_WORD        = 0x12          # LoRaRF turns this into 0x1424, same as the drone's C++

# Fragments arriving closer together than this belong to the same message
REASSEMBLY_GAP_SECONDS = 0.2

# How long to wait for the drone's reply. A command makes the drone transmit
# "ACK <id> <cmd>" and then (status) a "[Drone N] ..." line: two packets on air.
MAX_WAIT_SECONDS = 4.0

# Commands that must never queue behind a background status poll.
EMERGENCY_COMMANDS = {"land", "l", "stop", "estop"}

# Commands whose real answer is a telemetry line sent after the ACK.
TELEMETRY_COMMANDS = {"status"}

request_lock = threading.Lock()   # one command/reply exchange at a time


# ============================================================
# Radio backends. Both expose: send(text), lines (Queue of received
# complete lines), ok (bool), error (str), name (str)
# ============================================================
class SpiRadio:
    """SX1262 on the Pi's SPI pins, driven the same way as lora-c.py."""

    def __init__(self):
        self.name = "SX1262 (SPI)"
        self.lines = queue.Queue()
        self.ok = False
        self.error = ""
        self._lock = threading.Lock()
        try:
            from LoRaRF import SX126x
        except ImportError:
            self.error = "LoRaRF not installed: pip install LoRaRF"
            print(f"[!] {self.error}")
            return

        self.LoRa = SX126x()
        print("[*] Initializing SX1262...")
        if not self.LoRa.begin(SPI_BUS, SPI_CS, PIN_RESET, PIN_BUSY, PIN_IRQ, PIN_TXEN, PIN_RXEN):
            self.error = ("LoRa init failed - check wiring, that SPI is enabled (raspi-config), "
                          "and that lora-c.py / another program isn't still running")
            print(f"[!] {self.error}")
            return

        self.LoRa.setFrequency(FREQUENCY)
        self.LoRa.setTxPower(TX_POWER, self.LoRa.TX_POWER_SX1262)
        self.LoRa.setLoRaModulation(SPREADING_FACTOR, BANDWIDTH, CODING_RATE)
        self.LoRa.setLoRaPacket(self.LoRa.HEADER_EXPLICIT, PREAMBLE_LEN, 255, crcType=True)
        self.LoRa.setSyncWord(SYNC_WORD)
        self.ok = True
        print(f"[*] Radio ready | {FREQUENCY / 1e6} MHz | SF{SPREADING_FACTOR} | "
              f"{BANDWIDTH / 1000:.0f} kHz | {TX_POWER} dBm")

        threading.Thread(target=self._receive_loop, daemon=True).start()

    def _rx(self):
        self.LoRa.request(self.LoRa.RX_CONTINUOUS)

    def send(self, text):
        # Same as lora-c.py send_message(): append "\n", transmit, back to RX.
        data = list((text + "\n").encode())
        with self._lock:
            self.LoRa.beginPacket()
            self.LoRa.write(data, len(data))
            self.LoRa.endPacket()
            self.LoRa.wait()
            self._rx()

    def _emit(self, parts):
        text = "".join(parts)
        for line in text.replace("\r", "\n").split("\n"):
            line = line.strip()
            if line:
                print(f"[RX] {line}")
                self.lines.put(line)

    def _receive_loop(self):
        # Continuous RX + reassembly, ported from lora-c.py receive_loop().
        with self._lock:
            self._rx()
        parts = []
        last_time = None
        while True:
            try:
                with self._lock:
                    status = self.LoRa.status()
                    length = self.LoRa.available()
                    if status == 7 and length:          # STATUS_RX_DONE
                        fragment = bytes(self.LoRa.read(length)).decode(errors="replace")
                        now = time.time()
                        if parts and last_time is not None and now - last_time <= REASSEMBLY_GAP_SECONDS:
                            parts.append(fragment)
                        else:
                            if parts:
                                self._emit(parts)
                            parts = [fragment]
                        last_time = now
                        self._rx()
                # A newline means the message is complete - don't wait for the gap.
                if parts and "\n" in parts[-1]:
                    self._emit(parts)
                    parts, last_time = [], None
                elif parts and last_time is not None and time.time() - last_time > REASSEMBLY_GAP_SECONDS:
                    self._emit(parts)
                    parts, last_time = [], None
            except Exception as e:
                print(f"[!] RX error: {e}")
                try:
                    with self._lock:
                        self._rx()
                except Exception:
                    pass
                time.sleep(0.2)
            time.sleep(0.02)


class SerialRadio:
    """USB / UART LoRa module (transparent mode)."""

    def __init__(self, port, baud):
        import serial
        self.serial = serial
        self.name = "serial module"
        self.lines = queue.Queue()
        self.ok = False
        self.error = ""
        self.port = port or self._find_port()
        self.baud = baud
        self.ser = None
        self._wlock = threading.Lock()
        self._connect()
        threading.Thread(target=self._receive_loop, daemon=True).start()

    @staticmethod
    def _find_port():
        for pattern in ('/dev/ttyUSB*', '/dev/ttyACM*'):
            found = sorted(glob.glob(pattern))
            if found:
                return found[0]
        for dev in ('/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0'):
            if os.path.exists(dev):
                return dev
        return None

    def _connect(self):
        if not self.port:
            self.error = "no serial port found"
            return
        try:
            s = self.serial.Serial()
            s.port, s.baudrate, s.timeout = self.port, self.baud, 0.5
            s.dtr = False
            s.rts = False
            s.open()
            s.reset_input_buffer()
            self.ser, self.ok, self.error = s, True, ""
            self.name = f"serial module on {self.port} @ {self.baud}"
            print(f"[*] Connected to {self.name}")
        except Exception as e:
            self.ser, self.ok = None, False
            self.error = f"{self.port}: {e}"
            print(f"[!] Serial error: {self.error}")

    def send(self, text):
        with self._wlock:
            if not self.ok:
                self._connect()
            if not self.ok:
                raise RuntimeError(self.error)
            try:
                self.ser.write(f"{text}\n".encode())
                self.ser.flush()
            except Exception as e:
                self.ok, self.error = False, str(e)
                raise

    def _receive_loop(self):
        while True:
            if not self.ok:
                time.sleep(2)
                with self._wlock:
                    self._connect()
                continue
            try:
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    print(f"[RX] {line}")
                    self.lines.put(line)
            except Exception as e:
                self.ok, self.error = False, str(e)


radio = None


# ============================================================
# Helpers
# ============================================================
def command_action(cmd):
    """'1-land' -> 'land', '1-t 20' -> 't'"""
    action = cmd.split('-', 1)[1] if '-' in cmd else cmd
    return action.strip().split(' ')[0].lower()


def command_target(cmd):
    """'2-land' -> '2', 'all-land' -> 'all'"""
    return cmd.split('-', 1)[0].strip() if '-' in cmd else ''


def drain(q):
    while True:
        try:
            q.get_nowait()
        except queue.Empty:
            return


@app.route('/radio')
def radio_status():
    return jsonify({"ok": bool(radio and radio.ok),
                    "radio": radio.name if radio else None,
                    "error": radio.error if radio else "not started"})


@app.route('/send', methods=['POST', 'GET'])
def send_command():
    if radio is None or not radio.ok:
        return jsonify({"error": f"Radio not ready ({radio.error if radio else 'not started'})"}), 500

    if request.method == 'POST':
        data = request.get_json(silent=True)
        cmd = data.get('cmd') if data else request.form.get('cmd')
    else:
        cmd = request.args.get('cmd')

    if not cmd:
        return jsonify({"error": "No command provided. Send JSON {'cmd': '1-status'}"}), 400

    cmd = cmd.strip()
    target = command_target(cmd)
    if not target:
        return jsonify({"error": "Command must be '<drone id>-<cmd>' or 'all-<cmd>'"}), 400

    action = command_action(cmd)

    # Broadcast: emergency only, sent immediately, never awaited
    # (several drones answering at once would just collide on air).
    if target.lower() == 'all':
        if action not in EMERGENCY_COMMANDS | {"rtl"}:
            return jsonify({"error": "Only land / stop / estop / rtl can be sent to all drones"}), 400
        try:
            radio.send(cmd)
        except Exception as e:
            return jsonify({"error": str(e)}), 500
        return jsonify({"status": "success", "command": cmd,
                        "response": "BROADCAST SENT to all drones - replies not awaited"}), 200

    # Emergency commands jump the queue if a status poll is waiting for its reply.
    if action in EMERGENCY_COMMANDS and not request_lock.acquire(blocking=False):
        try:
            radio.send(cmd)
        except Exception as e:
            return jsonify({"error": str(e)}), 500
        return jsonify({"status": "success", "command": cmd,
                        "response": "SENT (priority) - reply not awaited"}), 200
    elif action not in EMERGENCY_COMMANDS:
        request_lock.acquire()
    # (an emergency command that got the lock immediately falls through, lock held)

    try:
        drain(radio.lines)          # forget stale replies
        radio.send(cmd)

        # The drone answers "ACK <id> <cmd>"; for status a "[Drone N] ..." line follows.
        # Several drones share the channel, so only accept replies from this one.
        wants_telemetry = action in TELEMETRY_COMMANDS
        deadline = time.time() + MAX_WAIT_SECONDS
        ack = ""
        response = ""
        while time.time() < deadline:
            try:
                line = radio.lines.get(timeout=max(0.01, deadline - time.time()))
            except queue.Empty:
                break
            if line.startswith(f"ACK {target} "):
                ack = line
                if not wants_telemetry:
                    break
            elif line.startswith(f"[Drone {target}]"):
                response = line
                break

        if response:
            return jsonify({"status": "success", "command": cmd, "response": response, "ack": ack}), 200
        if ack:
            return jsonify({"status": "success", "command": cmd, "response": ack, "ack": ack}), 200
        return jsonify({"status": "timeout", "command": cmd,
                        "message": "No response from drone (is it running with --lora-ack?)"}), 408

    except Exception as e:
        return jsonify({"error": str(e)}), 500
    finally:
        request_lock.release()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="LoRa ground station web relay")
    parser.add_argument("--serial", nargs="?", const="", metavar="PORT",
                        help="use a USB/UART LoRa module instead of the SX1262 on SPI "
                             "(port optional, auto-detected)")
    parser.add_argument("--baud", type=int, default=9600, help="serial module baud (default 9600)")
    parser.add_argument("--host", default="0.0.0.0", help="use 127.0.0.1 to allow only this computer")
    parser.add_argument("--web-port", type=int, default=5000)
    args = parser.parse_args()

    if args.serial is not None:
        radio = SerialRadio(args.serial or None, args.baud)
    else:
        radio = SpiRadio()

    print(f"[*] Web UI: http://<this-pi-ip>:{args.web_port}   (radio status: /radio)")
    # threaded=True so an emergency LAND request isn't stuck behind a poll request.
    app.run(host=args.host, port=args.web_port, threaded=True)

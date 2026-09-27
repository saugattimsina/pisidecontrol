import argparse
import glob
import os
import threading
import time

import serial
import serial.tools.list_ports
from flask import Flask, request, jsonify, render_template

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

# Serial port of the ESP32 LoRa bridge.
#   None  -> auto-detect (CP210x / CH340 / FTDI / ESP32-S3 native USB, then any ttyUSB*/ttyACM*)
#   Override with:  python3 flask_lora.py --port /dev/ttyACM0   or   LORA_PORT=/dev/ttyACM0
SERIAL_PORT = os.environ.get('LORA_PORT') or None
BAUD_RATE = int(os.environ.get('LORA_BAUD', '115200'))

# USB vendor IDs of common ESP32 USB-serial chips
ESP32_USB_VIDS = {0x10C4, 0x1A86, 0x0403, 0x303A}
RECONNECT_INTERVAL_S = 2.0
_last_connect_try = 0.0
last_serial_error = "not opened yet"

# How long to wait for the drone's reply. Each command makes the drone transmit
# an "ACK <cmd>" packet first and then (for status-type commands) a
# "[Drone N] ..." telemetry line, so allow time for two packets on air.
MAX_WAIT_SECONDS = 4.0

# Commands that must never queue behind a background status poll.
# (The drone's lora_link.cpp also lets land / stop / estop bypass its own queue.)
EMERGENCY_COMMANDS = {"land", "stop", "estop"}

# Commands whose real answer is a telemetry line sent after the ACK.
# Every other command returns as soon as its ACK arrives.
TELEMETRY_COMMANDS = {"status", "sensors"}

# Global serial object and a lock to prevent concurrent access by multiple HTTP requests
ser = None
serial_lock = threading.Lock()
write_lock = threading.Lock()   # guards ser.write() only, so emergencies can jump the queue


def find_serial_port():
    """Pick the ESP32's serial port."""
    if SERIAL_PORT:
        return SERIAL_PORT
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid in ESP32_USB_VIDS:
            return p.device
    for pattern in ('/dev/ttyUSB*', '/dev/ttyACM*'):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


def init_serial():
    """Open the ESP32 port. Safe to call repeatedly; retries at most every 2 s."""
    global ser, _last_connect_try, last_serial_error
    if ser is not None and ser.is_open:
        return True
    now = time.time()
    if now - _last_connect_try < RECONNECT_INTERVAL_S:
        return False
    _last_connect_try = now

    port = find_serial_port()
    if not port:
        last_serial_error = "no /dev/ttyUSB* or /dev/ttyACM* found - is the ESP32 plugged in?"
        print(f"[!] {last_serial_error}")
        return False
    try:
        s = serial.Serial()
        s.port = port
        s.baudrate = BAUD_RATE
        s.timeout = 0.5
        # Don't pulse DTR/RTS on open: on most ESP32 boards that resets the chip,
        # and the first commands would be lost while it reboots.
        s.dtr = False
        s.rts = False
        s.open()
        time.sleep(0.3)
        s.reset_input_buffer()          # drop any boot banner
        ser = s
        last_serial_error = ""
        print(f"[*] Connected to ESP32 LoRa bridge on {port} at {BAUD_RATE} baud.")
        return True
    except serial.SerialException as e:
        msg = str(e)
        if "Permission denied" in msg:
            hint = "add yourself to the dialout group: sudo usermod -aG dialout $USER, then log out/in"
        elif "busy" in msg.lower() or "Device or resource busy" in msg:
            hint = "another program has the port open (Arduino serial monitor, simple_lora, ground_station.py?)"
        else:
            hint = "check the USB cable / port"
        last_serial_error = f"{port}: {msg} -> {hint}"
        print(f"[!] Serial error: {last_serial_error}")
        ser = None
        return False


def drop_serial():
    """Forget a port that failed mid-use so the next request reconnects."""
    global ser
    try:
        if ser is not None:
            ser.close()
    except Exception:
        pass
    ser = None


def serial_watchdog():
    """Keep trying to (re)connect in the background, e.g. after unplug/replug."""
    while True:
        if ser is None or not ser.is_open:
            with write_lock:
                init_serial()
        time.sleep(RECONNECT_INTERVAL_S)


@app.route('/serial')
def serial_status():
    connected = ser is not None and ser.is_open
    return jsonify({"connected": connected,
                    "port": ser.port if connected else find_serial_port(),
                    "error": "" if connected else last_serial_error})


def write_line(cmd):
    with write_lock:
        try:
            ser.write(f"{cmd}\n".encode('utf-8'))
            ser.flush()
        except (serial.SerialException, OSError):
            drop_serial()
            raise


def command_action(cmd):
    """'1-land' -> 'land', '1-t 20' -> 't'"""
    action = cmd.split('-', 1)[1] if '-' in cmd else cmd
    return action.strip().split(' ')[0].lower()


def command_target(cmd):
    """'2-land' -> '2', 'all-land' -> 'all'"""
    return cmd.split('-', 1)[0].strip() if '-' in cmd else ''


@app.route('/send', methods=['POST', 'GET'])
def send_command():
    if ser is None or not ser.is_open:
        with write_lock:
            init_serial()
    if ser is None or not ser.is_open:
        return jsonify({"error": f"Serial port not connected ({last_serial_error})"}), 500

    # Get the command from query string (GET) or JSON/Form (POST)
    if request.method == 'POST':
        data = request.get_json(silent=True)
        cmd = data.get('cmd') if data else request.form.get('cmd')
    else:
        cmd = request.args.get('cmd')

    if not cmd:
        return jsonify({"error": "No command provided. Use ?cmd=1-arm or send JSON {'cmd': '1-arm'}"}), 400

    cmd = cmd.strip()
    target = command_target(cmd)
    if not target:
        return jsonify({"error": "Command must be '<drone id>-<cmd>' or 'all-<cmd>'"}), 400

    # Broadcast to every drone: emergency only, sent immediately, never awaited
    # (several drones answering at once would just collide on air).
    if target.lower() == 'all':
        if command_action(cmd) not in EMERGENCY_COMMANDS | {"rtl"}:
            return jsonify({"error": "Only land / stop / estop / rtl can be sent to all drones"}), 400
        try:
            write_line(cmd)
        except Exception as e:
            return jsonify({"error": str(e)}), 500
        return jsonify({"status": "success", "command": cmd,
                        "response": "BROADCAST SENT to all drones - replies not awaited"}), 200

    # Emergency commands go out immediately, even while a poll is waiting for its reply.
    if command_action(cmd) in EMERGENCY_COMMANDS and not serial_lock.acquire(blocking=False):
        try:
            write_line(cmd)
        except Exception as e:
            return jsonify({"error": str(e)}), 500
        return jsonify({"status": "success", "command": cmd,
                        "response": "SENT (priority) - reply not awaited"}), 200
    elif command_action(cmd) not in EMERGENCY_COMMANDS:
        serial_lock.acquire()
    # (emergency command that got the lock without waiting falls through here, lock held)

    try:
        # Clear out any stale data sitting in the receiver buffer
        ser.reset_input_buffer()

        # 1. Transmit the command over LoRa
        write_line(cmd)

        # 2. Wait for the reply. The drone sends "ACK <cmd>" first; for status-type
        #    commands the "[Drone N] ..." telemetry follows. Prefer the telemetry,
        #    other commands (arm, t 20, rtl...) return as soon as the ACK arrives.
        wants_telemetry = command_action(cmd) in TELEMETRY_COMMANDS
        start_time = time.time()
        ack = ""
        response = ""
        while (time.time() - start_time) < MAX_WAIT_SECONDS:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                # Several drones share the channel: ignore anything that is not
                # from the drone we just addressed.
                if line.startswith("ACK"):
                    if not line.startswith(f"ACK {target} "):
                        continue
                elif line.startswith("[Drone "):
                    if not line.startswith(f"[Drone {target}]"):
                        continue
                else:
                    continue           # unrelated / garbage line
                if line.startswith("ACK"):
                    ack = line
                    if not wants_telemetry:
                        break          # ACK is the whole answer - return immediately
                    continue           # status/sensors: keep waiting for the telemetry line
                response = line
                break
            time.sleep(0.05)

        if response:
            return jsonify({"status": "success", "command": cmd, "response": response, "ack": ack}), 200
        if ack:
            return jsonify({"status": "success", "command": cmd, "response": ack, "ack": ack}), 200
        return jsonify({"status": "timeout", "command": cmd, "message": "No response from drone"}), 408

    except (serial.SerialException, OSError) as e:
        drop_serial()                  # USB unplugged / ESP32 reset: reconnect next time
        return jsonify({"error": f"Serial link lost: {e}"}), 500
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    finally:
        serial_lock.release()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="LoRa ground station web relay (ESP32 SX1262 over USB)")
    parser.add_argument("--port", help="ESP32 serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--host", default="0.0.0.0", help="use 127.0.0.1 to allow only this computer")
    parser.add_argument("--web-port", type=int, default=5000)
    args = parser.parse_args()
    if args.port:
        SERIAL_PORT = args.port
    BAUD_RATE = args.baud

    print("[*] Serial ports seen:",
          ", ".join(f"{p.device} ({p.description})" for p in serial.tools.list_ports.comports()) or "none")
    init_serial()
    threading.Thread(target=serial_watchdog, daemon=True).start()

    print(f"[*] Web UI: http://localhost:{args.web_port}")
    # threaded=True so an emergency LAND request isn't stuck behind a poll request.
    app.run(host=args.host, port=args.web_port, threaded=True)

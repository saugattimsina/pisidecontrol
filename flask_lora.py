import serial
import threading
import time
from flask import Flask, request, jsonify, render_template

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

# Configure your serial port and baud rate here
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

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


def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        print(f"[*] Connected to LoRa module on {SERIAL_PORT} at {BAUD_RATE} baud.")
    except Exception as e:
        print(f"[!] Error connecting to serial port: {e}")
        print("[!] Make sure the device is plugged in and permissions are set.")


def write_line(cmd):
    with write_lock:
        ser.write(f"{cmd}\n".encode('utf-8'))
        ser.flush()


def command_action(cmd):
    """'1-land' -> 'land', '1-t 20' -> 't'"""
    action = cmd.split('-', 1)[1] if '-' in cmd else cmd
    return action.strip().split(' ')[0].lower()


@app.route('/send', methods=['POST', 'GET'])
def send_command():
    if ser is None or not ser.is_open:
        return jsonify({"error": "Serial port not connected"}), 500

    # Get the command from query string (GET) or JSON/Form (POST)
    if request.method == 'POST':
        data = request.get_json(silent=True)
        cmd = data.get('cmd') if data else request.form.get('cmd')
    else:
        cmd = request.args.get('cmd')

    if not cmd:
        return jsonify({"error": "No command provided. Use ?cmd=1-arm or send JSON {'cmd': '1-arm'}"}), 400

    cmd = cmd.strip()

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

    except Exception as e:
        return jsonify({"error": str(e)}), 500
    finally:
        serial_lock.release()


if __name__ == '__main__':
    init_serial()
    # Run the Flask app on all interfaces, port 5000.
    # threaded=True so an emergency LAND request isn't stuck behind a poll request.
    app.run(host='0.0.0.0', port=5000, threaded=True)

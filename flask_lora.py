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

# Global serial object and a lock to prevent concurrent access by multiple HTTP requests
ser = None
serial_lock = threading.Lock()

def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        print(f"[*] Connected to LoRa module on {SERIAL_PORT} at {BAUD_RATE} baud.")
    except Exception as e:
        print(f"[!] Error connecting to serial port: {e}")
        print("[!] Make sure the device is plugged in and permissions are set.")

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

    # The command should be raw, exactly as provided by the client (e.g., "1-arm")
    full_cmd = f"{cmd}\n"

    with serial_lock:
        try:
            # Clear out any stale data sitting in the receiver buffer
            ser.reset_input_buffer()
            
            # 1. Transmit the command over LoRa
            ser.write(full_cmd.encode('utf-8'))
            
            # 2. Wait for a response (e.g., "[Drone 1] A:Y|...")
            # We loop briefly to give the drone time to process and transmit back
            max_wait_seconds = 3.0
            start_time = time.time()
            response = ""
            
            while (time.time() - start_time) < max_wait_seconds:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        response = line
                        # If we get a response, we break immediately and return it
                        break
                time.sleep(0.1)
                
            if response:
                return jsonify({"status": "success", "command": cmd, "response": response}), 200
            else:
                return jsonify({"status": "timeout", "command": cmd, "message": "No response from drone"}), 408

        except Exception as e:
            return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    init_serial()
    # Run the Flask app on all interfaces, port 5000
    app.run(host='0.0.0.0', port=5000)

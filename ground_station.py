import serial
import threading
import time
import sys
import argparse

def parse_telemetry(line):
    """
    Parses a telemetry line like:
    [Drone 1] A:Y|M:GUIDED|Alt:10.5m|Baro:10.2m|Sats:8|Loc:40.7128,-74.0060|Trk:N
    """
    try:
        # Extract Drone ID
        end_bracket = line.index("]")
        drone_id = line[7:end_bracket]
        
        # Extract payload
        payload = line[end_bracket+2:].strip()
        parts = payload.split('|')
        
        print(f"\n🛸 --- Telemetry from Drone {drone_id} ---")
        for part in parts:
            if ':' in part:
                key, val = part.split(':', 1)
                labels = {
                    'A': 'Armed',
                    'M': 'Mode',
                    'Alt': 'Altitude',
                    'Baro': 'Barometer Alt',
                    'Sats': 'Satellites',
                    'Loc': 'Location (Lat,Lon)',
                    'Trk': 'Tracking Active'
                }
                label = labels.get(key, key)
                print(f"  {label}: {val}")
            else:
                print(f"  {part}")
        print("----------------------------------\n")
    except Exception as e:
        print(f"[!] Failed to parse telemetry line: {line}\nError: {e}")

def receive_loop(ser):
    while True:
        try:
            if ser.in_waiting > 0:
                # Read line and decode
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    if line.startswith("[Drone "):
                        parse_telemetry(line)
                    else:
                        print(f"[RAW RX]: {line}")
            time.sleep(0.05)
        except Exception as e:
            print(f"\n[!] Error reading from serial: {e}")
            break

def main():
    parser = argparse.ArgumentParser(description="Ground Station for LoRa Drone Telemetry")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port of the LoRa module")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--id", type=str, default="1", help="Target Drone ID to query")
    parser.add_argument("--poll", type=float, default=5.0, help="Polling interval in seconds (0 for manual mode only)")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"[*] Ground Station connected to LoRa on {args.port} at {args.baud} baud.")
    except Exception as e:
        print(f"[!] Error connecting to serial port: {e}")
        sys.exit(1)

    # Start the background receiver thread
    rx_thread = threading.Thread(target=receive_loop, args=(ser,), daemon=True)
    rx_thread.start()

    drone_id = args.id
    poll_interval = args.poll

    time.sleep(1) # wait for connection to settle

    try:
        if poll_interval > 0:
            print(f"[*] Auto-polling Drone {drone_id} every {poll_interval} seconds.")
            print("[*] Press Ctrl+C to exit.")
            while True:
                # Construct the command, e.g., "1-status\n"
                cmd = f"{drone_id}-status\n"
                ser.write(cmd.encode('utf-8'))
                time.sleep(poll_interval)
        else:
            print(f"[*] Manual mode. Type a command (e.g., 'status') and press Enter.")
            while True:
                user_input = input("")
                if user_input.lower() in ['quit', 'exit']:
                    break
                
                # If user just types 'status', prepend the drone ID automatically
                if not '-' in user_input:
                    cmd = f"{drone_id}-{user_input}\n"
                else:
                    cmd = f"{user_input}\n"
                    
                ser.write(cmd.encode('utf-8'))
                
    except KeyboardInterrupt:
        print("\n[*] Exiting Ground Station...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

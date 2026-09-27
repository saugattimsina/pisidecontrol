import serial
import threading
import time
import sys

# Configure your serial port and baud rate here
# Common ports on Linux/Raspberry Pi: '/dev/ttyUSB0', '/dev/ttyS0', '/dev/serial0'
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200  # Common baud rates are 9600, 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to LoRa module on {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"Error connecting to serial port: {e}")
    print("Please make sure the module is connected and you have permission to access the port.")
    sys.exit(1)

def receive_messages():
    """Continuously read from the serial port and print received messages."""
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(f"\n[Received]: {line}")
                    print("Enter message to send: ", end="", flush=True)
            time.sleep(0.1)
        except Exception as e:
            print(f"\nError reading from serial: {e}")
            break

def send_messages():
    """Read user input and send it to the serial port."""
    time.sleep(0.5) # Give receiver thread time to start
    while True:
        try:
            msg = input("Enter message to send (or 'quit' to exit): ")
            if msg.lower() == 'quit' or msg.lower() == 'exit':
                print("Exiting...")
                ser.close()
                sys.exit(0)
            
            # Send the message with a newline character
            # Note: Adjust the line ending (\r\n or \n) based on your specific LoRa module's AT command set if using one
            ser.write((msg + '\r\n').encode('utf-8'))
            print(f"[Sent]: {msg}")
        except KeyboardInterrupt:
            print("\nExiting...")
            ser.close()
            sys.exit(0)
        except Exception as e:
            print(f"Error sending message: {e}")
            break

if __name__ == '__main__':
    # Start the receiver thread
    receiver_thread = threading.Thread(target=receive_messages, daemon=True)
    receiver_thread.start()

    # Start sending messages from the main thread
    send_messages()

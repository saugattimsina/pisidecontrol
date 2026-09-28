#!/usr/bin/env python3
"""
find_gps.py - find the GPS connected to this computer and show your position.

    python3 find_gps.py                   # scan everything
    python3 find_gps.py /dev/ttyACM0      # check one port
    python3 find_gps.py --skip /dev/ttyUSB0   # don't touch this port (e.g. the LoRa module)

Stop flask_lora.py first if it uses a USB LoRa module: opening that port here would steal its data.
When it finds the GPS it prints the line to start Flask with, e.g.
    MY_GPS=/dev/ttyACM0 python3 flask_lora.py
"""
import sys
import time

import gps_reader as g


def main():
    args = sys.argv[1:]
    skip = []
    if "--skip" in args:
        i = args.index("--skip")
        skip = args[i + 1:i + 2]
        args = args[:i] + args[i + 2:]

    print("Looking for a GPS...")
    if args:
        port = args[0]
        baud, info = g.probe_port(port)
        if not baud:
            print(f"  {port}: {info}")
            return 1
        src = port
        print(f"  {port}: NMEA at {baud} baud")
    else:
        ports = g.candidate_ports(skip)
        print("  serial ports:", ", ".join(ports) if ports else "none")
        src, baud = g.find_gps(skip, verbose=True)
        if src is None:
            print("\nNo GPS found. Check:")
            print("  - is it plugged in?  ls /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/")
            print("  - permission denied? sudo usermod -aG dialout $USER  (then log out/in)")
            print("  - is gpsd or another program holding it? (sudo systemctl stop gpsd)")
            print("  - u-blox set to binary (UBX) only? enable NMEA output in u-center")
            return 1

    print(f"\nGPS found: {src}" + (f" at {baud} baud" if baud else ""))
    print(f"Start Flask with:  MY_GPS={src} python3 flask_lora.py\n")
    print("Reading position for 20 s (take it outside or near a window for a fix)...")
    r = g.GpsReader(src, baud).start()
    end = time.time() + 20
    while time.time() < end:
        time.sleep(2)
        s = r.snapshot()
        if s["error"]:
            print("  error:", s["error"])
        elif s["fix"] and s["lat"] is not None:
            print(f"  FIX  {s['lat']:.6f}, {s['lon']:.6f}  alt {s['alt']} m  sats {s['sats']}  hdop {s['hdop']}")
        else:
            print(f"  no fix yet (sats {s['sats']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())

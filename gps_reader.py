"""
gps_reader.py - find and read a GPS on this computer (USB/serial NMEA receiver, or gpsd).

Used by find_gps.py (run it to locate the GPS) and by flask_lora.py (MY_GPS=... for the
dashboard's MY GPS button). Needs pyserial for serial receivers; gpsd needs nothing extra.
"""
import glob
import json
import os
import socket
import threading
import time

COMMON_BAUDS = (9600, 4800, 38400, 115200, 57600, 19200)


# ---------------------------------------------------------------------------
# NMEA parsing
# ---------------------------------------------------------------------------
def nmea_checksum_ok(line):
    """'$GPGGA,...*47' -> True when the XOR checksum matches."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(cs[:2], 16)
    except ValueError:
        return False


def _deg(value, hemi):
    """NMEA ddmm.mmmm / dddmm.mmmm + N/S/E/W -> signed decimal degrees."""
    if not value or not hemi:
        return None
    try:
        v = float(value)
    except ValueError:
        return None
    d = int(v // 100)
    deg = d + (v - d * 100) / 60.0
    return -deg if hemi in ("S", "W") else deg


def parse_nmea(line):
    """Return a dict for GGA / RMC sentences (any talker: GP, GN, GL, GA, BD), else None."""
    if not nmea_checksum_ok(line):
        return None
    f = line.strip()[1:].split("*")[0].split(",")
    kind = f[0][2:]
    try:
        if kind == "GGA" and len(f) >= 10:
            q = int(f[6] or 0)
            return {"type": "GGA", "lat": _deg(f[2], f[3]), "lon": _deg(f[4], f[5]),
                    "fix": q > 0, "fix_quality": q, "sats": int(f[7] or 0),
                    "hdop": float(f[8]) if f[8] else None,
                    "alt": float(f[9]) if f[9] else None}
        if kind == "RMC" and len(f) >= 9:
            return {"type": "RMC", "lat": _deg(f[3], f[4]), "lon": _deg(f[5], f[6]),
                    "fix": f[2] == "A",
                    "speed_ms": float(f[7]) * 0.514444 if f[7] else None,
                    "course_deg": float(f[8]) if f[8] else None}
    except (ValueError, IndexError):
        return None
    return None


# ---------------------------------------------------------------------------
# Finding the receiver
# ---------------------------------------------------------------------------
def candidate_ports(exclude=()):
    """Serial ports a USB/serial GPS is likely on (by-id names first: they say what the device is)."""
    excl = {os.path.realpath(p) for p in exclude if p}
    seen, out = set(), []
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*", "/dev/rfcomm*",
                    "/dev/ttyAMA*", "/dev/serial0", "/dev/ttyS[0-3]"):
        for p in sorted(glob.glob(pattern)):
            real = os.path.realpath(p)
            if real in excl or real in seen:
                continue
            seen.add(real)
            out.append(p)
    return out


def probe_port(port, bauds=COMMON_BAUDS, seconds=2.5):
    """Listen on port at each baud; return (baud, first_valid_sentence) or (None, reason)."""
    try:
        import serial
    except ImportError:
        return None, "pyserial not installed (pip install pyserial)"
    last_err = "no NMEA data"
    for baud in bauds:
        try:
            with serial.Serial(port, baud, timeout=0.5) as s:
                end = time.time() + seconds
                buf = b""
                while time.time() < end:
                    buf += s.read(256)
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode("ascii", errors="ignore").strip()
                        if line.startswith("$") and nmea_checksum_ok(line):
                            return baud, line
                    if len(buf) > 4096:
                        buf = buf[-512:]
        except Exception as e:                       # busy, permission denied, gone...
            last_err = str(e).splitlines()[0]
            if "ermission" in last_err or "busy" in last_err.lower():
                break                                # no point trying other bauds
    return None, last_err


def gpsd_available(host="127.0.0.1", port=2947, timeout=0.5):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def find_gps(exclude=(), verbose=False):
    """Return ('gpsd', None) or (port, baud), or (None, None) if nothing found."""
    if gpsd_available():
        if verbose:
            print("  gpsd is running on localhost:2947 -> using it")
        return "gpsd", None
    for p in candidate_ports(exclude):
        if verbose:
            print(f"  probing {p} ...", end="", flush=True)
        baud, info = probe_port(p)
        if verbose:
            print(f" NMEA at {baud} baud ({info[:40]})" if baud else f" no ({info})")
        if baud:
            return p, baud
    return None, None


# ---------------------------------------------------------------------------
# Background reader
# ---------------------------------------------------------------------------
class GpsReader:
    """Keeps the latest fix from a serial NMEA receiver or gpsd. Never raises."""

    def __init__(self, source="auto", baud=None, exclude=()):
        self.source, self.baud, self.exclude = source, baud, exclude
        self.lock = threading.Lock()
        self.state = {"lat": None, "lon": None, "alt": None, "sats": None, "fix": False,
                      "hdop": None, "updated": None, "device": None, "error": None}

    def snapshot(self):
        with self.lock:
            s = dict(self.state)
        s["age_s"] = round(time.time() - s["updated"], 1) if s["updated"] else None
        return s

    def _set(self, **kw):
        with self.lock:
            self.state.update(kw)

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()
        return self

    def _run(self):
        while True:
            try:
                src, baud = self.source, self.baud
                if src in ("", "auto"):
                    src, baud = find_gps(self.exclude)
                    if src is None:
                        self._set(error="no GPS found (plugged in? run find_gps.py)", device=None)
                        time.sleep(10)
                        continue
                if src == "gpsd":
                    self._read_gpsd()
                else:
                    self._read_serial(src, baud)
            except Exception as e:
                self._set(error=str(e).splitlines()[0])
            time.sleep(3)

    def _read_serial(self, port, baud):
        import serial
        if not baud:
            baud, info = probe_port(port)
            if not baud:
                raise RuntimeError(f"{port}: {info}")
        self._set(device=f"{port} @ {baud}", error=None)
        with serial.Serial(port, baud, timeout=1) as s:
            buf = b""
            while True:
                chunk = s.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    msg = parse_nmea(raw.decode("ascii", errors="ignore"))
                    if not msg:
                        continue
                    upd = {"updated": time.time(), "fix": msg["fix"]}
                    if msg["fix"] and msg["lat"] is not None and msg["lon"] is not None:
                        upd.update(lat=msg["lat"], lon=msg["lon"])
                    if msg["type"] == "GGA":
                        upd.update(sats=msg["sats"], hdop=msg["hdop"], alt=msg["alt"])
                    self._set(**upd)

    def _read_gpsd(self, host="127.0.0.1", port=2947):
        self._set(device="gpsd", error=None)
        with socket.create_connection((host, port), timeout=10) as s:
            s.sendall(b'?WATCH={"enable":true,"json":true}\n')
            f = s.makefile("r", encoding="utf-8", errors="ignore")
            for line in f:
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                if m.get("class") == "TPV":
                    ok = m.get("mode", 0) >= 2 and "lat" in m and "lon" in m
                    upd = {"updated": time.time(), "fix": ok}
                    if ok:
                        upd.update(lat=m["lat"], lon=m["lon"], alt=m.get("altMSL", m.get("alt")))
                    self._set(**upd)
                elif m.get("class") == "SKY" and isinstance(m.get("satellites"), list):
                    self._set(sats=sum(1 for sat in m["satellites"] if sat.get("used")))

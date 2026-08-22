#!/usr/bin/env python3
"""
ntrip_client.py - Minimal NTRIP rover client for an RTKBase caster.

Connects to an NTRIP mountpoint, reads the live RTCM3 correction stream,
validates each message (CRC-24Q), and prints a rolling dashboard of exactly
what a rover needs to obtain an RTK fix:

  - Base station coordinates (from RTCM 1005/1006)
  - Constellations present and satellite counts (from MSM observation messages)
  - Whether ephemeris is being broadcast
  - Message types, their rates, and correction age
  - Data throughput and CRC integrity

Standard library only. Defaults target the local RTKBase caster.

Usage:
  ./ntrip_client.py                         # local caster, mount RTK-BASE
  ./ntrip_client.py --host 192.168.1.10 --port 2101 --mount RTK-BASE
  ./ntrip_client.py --sourcetable           # just list mounts and exit
  ./ntrip_client.py -u user -p pass         # with basic auth
"""

import argparse
import base64
import math
import socket
import sys
import time
from collections import defaultdict

# --------------------------------------------------------------------------- #
# RTCM3 message-type names (the subset an RTK rover cares about)
# --------------------------------------------------------------------------- #
MSG_NAMES = {
    1004: "GPS L1/L2 obs (legacy)",
    1005: "Base ARP position (ECEF)",
    1006: "Base ARP position + height",
    1007: "Antenna descriptor",
    1008: "Antenna descriptor + serial",
    1012: "GLONASS L1/L2 obs (legacy)",
    1019: "GPS ephemeris",
    1020: "GLONASS ephemeris",
    1033: "Receiver & antenna descriptors",
    1042: "BeiDou ephemeris",
    1044: "QZSS ephemeris",
    1045: "Galileo F/NAV ephemeris",
    1046: "Galileo I/NAV ephemeris",
    1077: "GPS MSM7 obs",
    1087: "GLONASS MSM7 obs",
    1097: "Galileo MSM7 obs",
    1107: "SBAS MSM7 obs",
    1127: "BeiDou MSM7 obs",
    1230: "GLONASS code-phase biases",
}

# MSM observation message ranges -> constellation label
MSM_CONSTELLATION = {
    (1071, 1077): "GPS",
    (1081, 1087): "GLONASS",
    (1091, 1097): "Galileo",
    (1101, 1107): "SBAS",
    (1111, 1117): "QZSS",
    (1121, 1127): "BeiDou",
}
EPHEMERIS_TYPES = {1019, 1020, 1042, 1044, 1045, 1046}


def msg_name(mt):
    if mt in MSG_NAMES:
        return MSG_NAMES[mt]
    for (lo, hi), name in MSM_CONSTELLATION.items():
        if lo <= mt <= hi:
            return f"{name} MSM obs"
    return f"RTCM type {mt}"


def constellation_of(mt):
    for (lo, hi), name in MSM_CONSTELLATION.items():
        if lo <= mt <= hi:
            return name
    return None


# --------------------------------------------------------------------------- #
# CRC-24Q (used by RTCM3 for frame integrity)
# --------------------------------------------------------------------------- #
def crc24q(data):
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


# --------------------------------------------------------------------------- #
# Bit reader for RTCM payloads (MSB-first)
# --------------------------------------------------------------------------- #
class Bits:
    def __init__(self, data):
        self.d = data
        self.pos = 0

    def u(self, n):
        v = 0
        for _ in range(n):
            byte = self.d[self.pos >> 3]
            bit = (byte >> (7 - (self.pos & 7))) & 1
            v = (v << 1) | bit
            self.pos += 1
        return v

    def s(self, n):
        v = self.u(n)
        if v & (1 << (n - 1)):
            v -= 1 << n
        return v


def ecef_to_llh(x, y, z):
    """WGS84 ECEF (m) -> lat, lon (deg), height (m ellipsoidal)."""
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = f * (2 - f)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - e2))
    for _ in range(8):
        n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - e2 * n / (n + h)))
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), h


def decode_1005_1006(payload):
    """Return (lat, lon, height) from a type 1005/1006 message payload."""
    b = Bits(payload)
    b.u(12)          # message number
    b.u(12)          # reference station id
    b.u(6)           # ITRF realisation year
    b.u(4)           # indicator bits (GPS/GLO/GAL/reserved)
    x = b.s(38) * 1e-4
    b.u(2)           # single receiver / reserved
    y = b.s(38) * 1e-4
    b.u(2)
    z = b.s(38) * 1e-4
    return ecef_to_llh(x, y, z)


def decode_msm_sats(payload):
    """Return number of satellites in an MSM message (popcount of sat mask)."""
    b = Bits(payload)
    b.u(12 + 12 + 30 + 1 + 3 + 7 + 2 + 2 + 1 + 3)  # MSM header up to masks
    satmask = b.u(64)
    return bin(satmask).count("1")


# --------------------------------------------------------------------------- #
# NMEA parsing (rover output) - GGA gives the fix quality a rover reports
# --------------------------------------------------------------------------- #
# GGA field 6 "quality" -> human label. 4/5 are what RTK is all about.
FIX_QUALITY = {
    0: "NO FIX",
    1: "SINGLE (autonomous)",
    2: "DGPS",
    3: "PPS",
    4: "RTK FIXED",
    5: "RTK FLOAT",
    6: "DEAD RECKONING",
    7: "MANUAL",
    8: "SIMULATION",
}


def nmea_checksum_ok(line):
    if "*" not in line or not line.startswith("$"):
        return False
    body, _, cs = line[1:].partition("*")
    try:
        want = int(cs[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def _nmea_deg(val, hemi):
    """NMEA ddmm.mmmm + hemisphere -> signed decimal degrees."""
    if not val:
        return None
    dot = val.find(".")
    deg_len = dot - 2
    deg = float(val[:deg_len])
    minutes = float(val[deg_len:])
    dec = deg + minutes / 60.0
    if hemi in ("S", "W"):
        dec = -dec
    return dec


def parse_gga(line):
    """Parse a GGA sentence into a dict, or None if not GGA/invalid."""
    if "GGA" not in line[:6]:
        return None
    f = line.split("*")[0].split(",")
    if len(f) < 15:
        return None

    def num(x, cast=float):
        try:
            return cast(x)
        except (ValueError, TypeError):
            return None

    return {
        "quality": num(f[6], int) or 0,
        "sats": num(f[7], int),
        "hdop": num(f[8]),
        "lat": _nmea_deg(f[2], f[3]),
        "lon": _nmea_deg(f[4], f[5]),
        "alt": num(f[9]),
        "corr_age": num(f[13]),      # age of differential corrections (s)
        "station": f[14] if len(f) > 14 else "",
    }


def parse_gst(line):
    """Parse GST -> horizontal std dev (m), or None."""
    if "GST" not in line[:6]:
        return None
    f = line.split("*")[0].split(",")
    if len(f) < 9:
        return None
    try:
        lat_std = float(f[6])
        lon_std = float(f[7])
    except (ValueError, TypeError):
        return None
    return math.hypot(lat_std, lon_std)


# --------------------------------------------------------------------------- #
# NTRIP connection
# --------------------------------------------------------------------------- #
def ntrip_connect(host, port, mount, user, pwd, timeout=10):
    sock = socket.create_connection((host, port), timeout=timeout)
    req = (
        f"GET /{mount} HTTP/1.0\r\n"
        f"Host: {host}:{port}\r\n"
        f"Ntrip-Version: Ntrip/2.0\r\n"
        f"User-Agent: NTRIP python-rover/1.0\r\n"
    )
    if user is not None:
        token = base64.b64encode(f"{user}:{pwd or ''}".encode()).decode()
        req += f"Authorization: Basic {token}\r\n"
    req += "\r\n"
    sock.sendall(req.encode())

    # Read the response header. NTRIP-1 casters (e.g. str2str) reply
    # "ICY 200 OK\r\n" and then stream binary immediately, with NO blank line.
    # NTRIP-2/HTTP casters use a normal header block ending in "\r\n\r\n".
    sock.settimeout(timeout)
    head = b""
    boundary = None
    while boundary is None:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("connection closed during NTRIP handshake")
        head += chunk
        upper = head[:16].upper()
        if upper.startswith(b"HTTP") or upper.startswith(b"SOURCETABLE"):
            idx = head.find(b"\r\n\r\n")
            if idx >= 0:
                boundary = idx + 4
        else:  # ICY / NTRIP-1: header is just the status line
            idx = head.find(b"\r\n")
            if idx >= 0:
                boundary = idx + 2
                if head[boundary:boundary + 2] == b"\r\n":  # optional blank line
                    boundary += 2
    first = head.split(b"\r\n", 1)[0].decode(errors="replace")
    if "200" not in first:
        raise ConnectionError(f"Caster refused mount '{mount}': {first!r}")
    leftover = head[boundary:]          # first bytes of the RTCM stream
    return sock, first, leftover


def get_sourcetable(host, port, timeout=10):
    sock = socket.create_connection((host, port), timeout=timeout)
    req = (
        f"GET / HTTP/1.0\r\nHost: {host}:{port}\r\n"
        f"Ntrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIP python-rover/1.0\r\n\r\n"
    )
    sock.sendall(req.encode())
    sock.settimeout(3)
    data = b""
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            data += chunk
    except socket.timeout:
        pass
    sock.close()
    return data.decode(errors="replace")


# --------------------------------------------------------------------------- #
# Main streaming loop with rolling dashboard
# --------------------------------------------------------------------------- #
def run(args):
    if args.sourcetable:
        print(get_sourcetable(args.host, args.port))
        return 0

    print(f"Connecting to {args.host}:{args.port}/{args.mount} ...")
    try:
        sock, status, leftover = ntrip_connect(
            args.host, args.port, args.mount, args.user, args.password
        )
    except (OSError, ConnectionError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    print(f"Connected: {status}\n")
    sock.settimeout(5)

    buf = bytearray(leftover)
    counts = defaultdict(int)
    last_seen = {}
    sat_counts = {}          # constellation -> latest sat count
    base_llh = None
    crc_errors = 0
    total_msgs = 0
    total_bytes = len(leftover)
    start = time.monotonic()
    last_draw = 0.0

    is_tty = sys.stdout.isatty()

    def draw():
        elapsed = max(time.monotonic() - start, 1e-6)
        if is_tty:
            sys.stdout.write("\033[2J\033[H")   # clear screen for live view
        else:
            print("\n" + "=" * 64)             # plain separator when piped
        print(f"NTRIP rover view  ->  {args.host}:{args.port}/{args.mount}")
        print(f"uptime {elapsed:6.1f}s   {total_msgs} msgs   "
              f"{total_bytes/1024:.1f} KiB   "
              f"{total_bytes*8/elapsed/1000:.1f} kbit/s   "
              f"CRC errors: {crc_errors}")
        print("-" * 64)

        if base_llh:
            lat, lon, h = base_llh
            print(f"BASE POSITION : {lat:.8f}, {lon:.8f}   {h:.3f} m (ellipsoidal)")
        else:
            print("BASE POSITION : (waiting for RTCM 1005/1006 ...)")

        if sat_counts:
            total_sats = sum(sat_counts.values())
            parts = "  ".join(f"{k}:{v}" for k, v in sorted(sat_counts.items()))
            print(f"SATELLITES    : {total_sats} total   {parts}")
        else:
            print("SATELLITES    : (waiting for observation messages ...)")

        has_obs = any(constellation_of(mt) for mt in counts)
        has_eph = any(mt in EPHEMERIS_TYPES for mt in counts)
        ready = base_llh is not None and has_obs
        print(f"RTK READINESS : base_pos={'Y' if base_llh else 'N'}  "
              f"observations={'Y' if has_obs else 'N'}  "
              f"ephemeris={'Y' if has_eph else 'N'}  "
              f"-> {'READY for rover RTK' if ready else 'incomplete'}")
        print("-" * 64)
        print(f"{'type':>5}  {'count':>6}  {'rate':>6}  {'age':>5}  name")
        now = time.monotonic()
        for mt in sorted(counts):
            rate = counts[mt] / elapsed
            age = now - last_seen[mt]
            print(f"{mt:>5}  {counts[mt]:>6}  {rate:>5.1f}/s  {age:>4.1f}s  "
                  f"{msg_name(mt)}")
        sys.stdout.flush()

    try:
        while True:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                print("WARN: no data for 5s (caster stalled?)", file=sys.stderr)
                continue
            if not chunk:
                print("Stream closed by caster.", file=sys.stderr)
                break
            buf += chunk
            total_bytes += len(chunk)

            # Parse complete RTCM3 frames out of the buffer.
            i = 0
            while True:
                sync = buf.find(0xD3, i)
                if sync < 0 or sync + 3 > len(buf):
                    break
                length = ((buf[sync + 1] & 0x03) << 8) | buf[sync + 2]
                frame_end = sync + 3 + length + 3
                if frame_end > len(buf):
                    break  # wait for more bytes
                frame = bytes(buf[sync:frame_end])
                payload = frame[3:3 + length]
                crc_rx = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
                if crc24q(frame[:3 + length]) != crc_rx:
                    crc_errors += 1
                    i = sync + 1        # resync one byte forward
                    continue
                # Valid frame
                mt = (payload[0] << 4) | (payload[1] >> 4)
                counts[mt] += 1
                last_seen[mt] = time.monotonic()
                total_msgs += 1
                if mt in (1005, 1006):
                    try:
                        base_llh = decode_1005_1006(payload)
                    except Exception:
                        pass
                else:
                    con = constellation_of(mt)
                    if con:
                        try:
                            sat_counts[con] = decode_msm_sats(payload)
                        except Exception:
                            pass
                i = frame_end
            del buf[:i]

            if time.monotonic() - last_draw > args.interval:
                draw()
                last_draw = time.monotonic()
            if args.duration and time.monotonic() - start >= args.duration:
                draw()
                break
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()
    return 0


def run_rover(args):
    """
    Rover mode: pipe caster corrections INTO a locally-attached GNSS receiver
    and read its NMEA output back to report live fix quality.

        caster (RTCM3) --> [this client] --> rover receiver serial
        rover NMEA (GGA/GST) --> [this client] --> dashboard
    """
    try:
        import serial  # pyserial, only needed for rover mode
    except ImportError:
        print("ERROR: rover mode needs pyserial:  pip install pyserial",
              file=sys.stderr)
        return 1

    if args.rover_port == "/dev/ttyACM0":
        print("WARNING: /dev/ttyACM0 is the BASE receiver held by str2str_tcp; "
              "the rover must be a SEPARATE receiver on its own port.",
              file=sys.stderr)

    try:
        rover = serial.Serial(args.rover_port, args.rover_baud, timeout=0.5)
    except serial.SerialException as e:
        print(f"ERROR opening rover port {args.rover_port}: {e}", file=sys.stderr)
        return 1
    print(f"Rover port : {args.rover_port} @ {args.rover_baud}")

    print(f"Connecting to {args.host}:{args.port}/{args.mount} ...")
    try:
        sock, status, leftover = ntrip_connect(
            args.host, args.port, args.mount, args.user, args.password
        )
    except (OSError, ConnectionError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        rover.close()
        return 1
    print(f"Connected  : {status}\n")
    sock.settimeout(5)

    # Shared state updated by the feeder thread, read by the display loop.
    import threading
    state = {
        "rtcm_bytes": len(leftover),
        "rtcm_msgs": 0,
        "base_llh": None,
        "last_rtcm": time.monotonic() if leftover else 0.0,
        "running": True,
    }
    lock = threading.Lock()

    def feeder():
        # Write any handshake leftover immediately, then pump caster -> rover.
        if leftover:
            rover.write(leftover)
        buf = bytearray(leftover)
        while state["running"]:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            rover.write(chunk)                      # feed corrections to receiver
            buf += chunk
            with lock:
                state["rtcm_bytes"] += len(chunk)
                state["last_rtcm"] = time.monotonic()
            # Lightweight frame scan for stats + base position.
            i = 0
            while True:
                s = buf.find(0xD3, i)
                if s < 0 or s + 3 > len(buf):
                    break
                length = ((buf[s + 1] & 0x03) << 8) | buf[s + 2]
                end = s + 3 + length + 3
                if end > len(buf):
                    break
                payload = bytes(buf[s + 3:s + 3 + length])
                mt = (payload[0] << 4) | (payload[1] >> 4)
                with lock:
                    state["rtcm_msgs"] += 1
                    if mt in (1005, 1006):
                        try:
                            state["base_llh"] = decode_1005_1006(payload)
                        except Exception:
                            pass
                i = end
            del buf[:i]

    th = threading.Thread(target=feeder, daemon=True)
    th.start()

    fix = {"quality": 0, "sats": None, "hdop": None, "lat": None, "lon": None,
           "alt": None, "corr_age": None, "hstd": None, "last": 0.0}
    start = time.monotonic()
    last_draw = 0.0
    last_gga_sent = 0.0
    is_tty = sys.stdout.isatty()

    def draw():
        el = max(time.monotonic() - start, 1e-6)
        if is_tty:
            sys.stdout.write("\033[2J\033[H")
        else:
            print("\n" + "=" * 64)
        with lock:
            rb, rm, base, last_rtcm = (state["rtcm_bytes"], state["rtcm_msgs"],
                                       state["base_llh"], state["last_rtcm"])
        print(f"RTK ROVER  ->  caster {args.host}:{args.port}/{args.mount}"
              f"   rover {args.rover_port}")
        print("-" * 64)
        qname = FIX_QUALITY.get(fix["quality"], f"q{fix['quality']}")
        star = "  <<<" if fix["quality"] in (4, 5) else ""
        print(f"  FIX STATUS : {qname}{star}")
        if fix["lat"] is not None:
            print(f"  POSITION   : {fix['lat']:.8f}, {fix['lon']:.8f}   "
                  f"{fix['alt']} m")
        else:
            print("  POSITION   : (waiting for rover NMEA ...)")
        acc = f"{fix['hstd']:.3f} m" if fix["hstd"] is not None else "n/a"
        print(f"  SATS USED  : {fix['sats']}    HDOP: {fix['hdop']}    "
              f"H-accuracy: {acc}")
        print(f"  CORR AGE   : {fix['corr_age']} s (age of corrections at rover)")
        print("-" * 64)
        corr_dt = (time.monotonic() - last_rtcm) if last_rtcm else 999
        print(f"  CORRECTIONS: {rm} msgs   {rb/1024:.1f} KiB   "
              f"{rb*8/el/1000:.1f} kbit/s   last {corr_dt:.1f}s ago")
        if base:
            print(f"  BASE POS   : {base[0]:.8f}, {base[1]:.8f}   {base[2]:.3f} m")
        flowing = corr_dt < 3
        print(f"  LINK       : corrections {'FLOWING' if flowing else 'STALLED'}"
              f"   rover fix {'OK' if fix['quality'] in (4, 5) else 'not yet RTK'}")
        sys.stdout.flush()

    try:
        while True:
            raw = rover.readline()
            if raw:
                line = raw.decode("ascii", "replace").strip()
                if nmea_checksum_ok(line):
                    g = parse_gga(line)
                    if g:
                        fix.update(g)
                        fix["last"] = time.monotonic()
                        # Optionally push our GGA up to the caster (VRS networks).
                        if args.send_gga and time.monotonic() - last_gga_sent > 10:
                            try:
                                sock.sendall((line + "\r\n").encode())
                                last_gga_sent = time.monotonic()
                            except OSError:
                                pass
                    else:
                        s = parse_gst(line)
                        if s is not None:
                            fix["hstd"] = s
            now = time.monotonic()
            if now - last_draw > args.interval:
                draw()
                last_draw = now
            if args.duration and now - start >= args.duration:
                draw()
                break
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        state["running"] = False
        try:
            sock.close()
        except OSError:
            pass
        rover.close()
    return 0


def main():
    try:
        sys.stdout.reconfigure(line_buffering=True)   # flush on newline when piped
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="NTRIP rover client / RTCM3 monitor")
    ap.add_argument("--host", default="192.168.1.10", help="caster host")
    ap.add_argument("--port", type=int, default=2101, help="caster port")
    ap.add_argument("--mount", default="RTK-BASE", help="mountpoint")
    ap.add_argument("-u", "--user", default=None, help="username (if auth)")
    ap.add_argument("-p", "--password", default=None, help="password (if auth)")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="dashboard refresh seconds")
    ap.add_argument("--duration", type=float, default=0,
                    help="stop after N seconds (0 = run until Ctrl-C)")
    ap.add_argument("--sourcetable", action="store_true",
                    help="print caster sourcetable and exit")
    rg = ap.add_argument_group("rover mode (feed a locally-attached receiver)")
    rg.add_argument("--rover-port", default=None,
                    help="serial device of the rover receiver, e.g. /dev/ttyACM1")
    rg.add_argument("--rover-baud", type=int, default=115200,
                    help="rover serial baud (default 115200)")
    rg.add_argument("--send-gga", action="store_true",
                    help="send rover GGA up to the caster (needed for VRS networks)")
    args = ap.parse_args()
    if args.rover_port:
        sys.exit(run_rover(args))
    sys.exit(run(args))


if __name__ == "__main__":
    main()

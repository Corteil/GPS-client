# GPS-client

[![CI](https://github.com/Corteil/GPS-client/actions/workflows/ci.yml/badge.svg)](https://github.com/Corteil/GPS-client/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.9%2B-blue.svg)](https://www.python.org/)

A minimal, dependency-free **NTRIP rover client** for inspecting an RTKBase
caster. It connects to a mountpoint, reads the live RTCM3 correction stream,
validates each message with CRC-24Q, and prints a rolling dashboard of exactly
what a rover needs to obtain an RTK fix.

## What it shows

- **Base station position** decoded from RTCM 1005/1006 (ECEF → lat/lon/height)
- **Constellations and satellite counts** from the MSM observation messages
- **RTK readiness** — whether base position + observations are present
- **Message inventory** — each RTCM type, its rate, and time since last seen
- **Throughput and CRC integrity** of the stream

## Usage

```bash
# Local RTKBase caster (defaults: 192.168.1.10:2101, mount RTK-BASE, no auth)
./ntrip_client.py

# Explicit target
./ntrip_client.py --host 192.168.1.10 --port 2101 --mount RTK-BASE

# With basic auth
./ntrip_client.py -u myuser -p mypass

# List the caster's mountpoints and exit
./ntrip_client.py --sourcetable

# Run for a fixed time (headless / logging), refresh every 2s
./ntrip_client.py --duration 30 --interval 2
```

### Rover mode — feed a locally-attached receiver

Pipes the caster's corrections into a second GNSS receiver on a serial port and
reads back its live fix quality (Single / DGPS / RTK-Float / RTK-Fixed).
Requires `pyserial`.

```bash
# Rover receiver on /dev/ttyACM1 (a SEPARATE receiver from the base)
./ntrip_client.py --rover-port /dev/ttyACM1 --rover-baud 115200

# For a VRS / networked caster that requires a client position upstream:
./ntrip_client.py --rover-port /dev/ttyACM1 --send-gga
```

The dashboard shows `FIX STATUS`, position, sats used, HDOP, correction age,
and confirms corrections are flowing into the receiver.

## Guides

- **FRESH-INSTALL.md** — build the whole base + rover system from a blank SD
  card (Raspberry Pi OS → RTKBase → receiver config → rover).
- **SETUP-GUIDE.md** — reference for the running system: concepts, data-flow
  diagram, fix-status table, and troubleshooting.

Press `Ctrl-C` to stop. Requires only Python 3 (standard library).

## Example output

```
NTRIP rover view  ->  192.168.1.10:2101/RTK-BASE
uptime   12.2s   91 msgs   8.4 KiB   5.6 kbit/s   CRC errors: 0
----------------------------------------------------------------
BASE POSITION : 51.47700000, -0.00140000   46.000 m (ellipsoidal)
SATELLITES    : 17 total   GLONASS:8  GPS:9
RTK READINESS : base_pos=Y  observations=Y  ephemeris=N  -> READY for rover RTK
----------------------------------------------------------------
 type   count    rate    age  name
 1005       2    0.2/s   1.9s  Base ARP position (ECEF)
 1077      12    1.0/s   1.0s  GPS MSM7 obs
 1087      12    1.0/s   1.0s  GLONASS MSM7 obs
 ...
```

## Testing

Unit tests cover the pure (non-network) decoding logic in `ntrip_client.py`.
They use only the standard library, so no fixtures or hardware are needed:

```bash
python3 -m unittest -v test_ntrip_client.py
```

`test_ntrip_client.py` contains 13 tests in four groups:

| Group | What it checks |
|-------|----------------|
| `CRC24Q` | CRC-24Q validation against a **real captured RTCM3 1077 frame** (header+payload CRC equals the trailing 3 bytes), detection of a flipped byte, and the empty-input case |
| `Ecef` | `ecef_to_llh` ECEF→lat/lon/height conversion at the equator/prime-meridian and a typical mid-latitude surface point |
| `Nmea` | NMEA checksum validation, GGA parsing (RTK-fixed quality, sat count, correction age), S/W sign handling, GST accuracy, and rejection of non-GGA sentences |
| `MsgNames` | RTCM message-type → constellation mapping and human-readable names |

These run automatically in CI (see the badge above) on Python 3.9–3.13 for
every push and pull request, alongside a `py_compile` check of the client.

## Notes

- `ephemeris=N` is normal and does **not** block RTK — a rover gets ephemeris
  from its own receiver. The base only needs to send observations + its
  position. Ephemeris messages (1019/1020/…) appear intermittently.
- This is a **monitor/diagnostic** client: it verifies the correction feed.
  A real positioning rover feeds this same RTCM3 stream into its GNSS receiver
  (or an RTK engine such as `rtkrcv`/`str2str`) alongside its own observations.

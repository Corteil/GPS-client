# RTK GNSS — Complete Base + Rover Setup Guide

End-to-end guide for the centimetre-accurate RTK system running on this
Raspberry Pi: a **fixed base station** that broadcasts corrections over a local
NTRIP caster, and a **rover** that consumes them to compute an RTK fix.

---

## 1. How RTK works (the 30-second version)

A single GNSS receiver is accurate to a few metres. RTK (Real-Time Kinematic)
gets to ~1–2 cm by using **two** receivers:

- A **base** at a fixed, known location measures the GNSS carrier phase and
  computes the errors (atmosphere, orbits, clocks) common to its area. It
  broadcasts these as **RTCM3 correction messages**.
- A **rover** applies those corrections to its own measurements and solves the
  short baseline to the base, yielding a position accurate to centimetres
  *relative to the base*.

The link between them is **NTRIP** — HTTP-like streaming of RTCM3 over TCP,
brokered by a **caster** with named **mountpoints**.

```
   [ Base GNSS antenna ]                             [ Rover GNSS antenna ]
           |                                                   |
     u-blox NEO-M8P (base)                          u-blox rover (M8P/F9P)
           | USB (raw UBX: RXM-RAWX + RXM-SFRBX)               ^ RTCM3 in
           v                                                   | NMEA out
   str2str_tcp  --TCP:5015-->  str2str_local_ntrip_caster      |
                                         |                      |
                                   NTRIP caster                 |
                              tcp/2101  mount "RTK-BASE"        |
                                         |                      |
                                         +----- LAN / Internet -+
                                                (RTCM3 corrections)
```

On this Pi the base pipeline **and** the caster run together; the rover is any
receiver that connects to the caster (including via the `ntrip_client.py`
rover mode in this folder).

---

## 2. Requirements checklist

### Base station (this Pi)
| Item | This build | Notes |
|------|-----------|-------|
| SBC | Raspberry Pi CM5 Lite, Debian 13 | Any Pi 4/5 works |
| GNSS receiver | u-blox **NEO-M8P-2**, HPG 1.40 firmware | Must support raw output (RXM-RAWX/SFRBX) |
| Antenna | Multi-GNSS active antenna on a **fixed, sky-open mount** | Stability matters more than cost |
| Connection | USB → `/dev/ttyACM0` @ 115200 | |
| Software | RTKBase + RTKLIB (`str2str`) + `gpsd` | Already installed under `/home/pi/rtkbase` |
| Network | LAN IP `192.168.1.10` | Static/reserved IP recommended |

### Rover
| Item | Requirement |
|------|-------------|
| GNSS receiver | u-blox rover capable of RTK — **ZED-F9P recommended** (dual-band, fast fix); NEO-M8P-0/2 works (single-band) |
| Antenna | Multi-GNSS active antenna with a clear sky view |
| Connection | Serial/USB to the machine running the client (e.g. `/dev/ttyACM1`) |
| Corrections | Network path to the caster (`192.168.1.10:2101`), and the base within ~10–30 km |
| Software | This `ntrip_client.py` (rover mode), **or** any NTRIP rover app (u-center, SW Maps, Emlid, etc.) |

### Both
- Both antennas need an **open view of the sky**; RTK needs common satellites.
- Base–rover **baseline** short enough for the correction type (single-band M8P:
  keep under ~10 km for reliable fixes; dual-band F9P tolerates longer).

---

## 3. Base station setup

> The base on this Pi is already built and running. This section documents the
> configuration so it can be reproduced or repaired.

### 3.1 Packages
```bash
# RTKLIB tools (str2str, rtkrcv) and gpsd clients
#   str2str / rtkrcv are in /usr/local/bin (built from RTKLIB)
sudo apt install gpsd gpsd-clients python3-serial
# RTKBase itself lives in /home/pi/rtkbase (web UI + systemd units)
```

### 3.2 Receiver configuration — the critical step
The M8P must output **UBX raw** messages so the pipeline can build a *complete*
RTCM stream (observations **and** ephemeris) and so the web UI's `rtkrcv` can
compute a live position:

- `RXM-RAWX`  (0x02 0x15) — raw carrier-phase / pseudorange observations
- `RXM-SFRBX` (0x02 0x13) — broadcast navigation frames (ephemeris)

Enable both and **save to flash** so they survive a power cycle. With the base
services stopped (to free the serial port):

```bash
sudo systemctl stop str2str_tcp str2str_local_ntrip_caster
python3 - <<'PY'
import serial, time
def ck(p):
    a=b=0
    for x in p: a=(a+x)&0xFF; b=(b+a)&0xFF
    return bytes([a,b])
def msg(cls,mid,pl):
    body=bytes([cls,mid,len(pl)&0xFF,(len(pl)>>8)&0xFF])+pl
    return b'\xb5\x62'+body+ck(body)
s=serial.Serial('/dev/ttyACM0',115200,timeout=1)
s.write(msg(0x06,0x01,bytes([0x02,0x15,0x01])))   # RXM-RAWX  @ 1 Hz
s.write(msg(0x06,0x01,bytes([0x02,0x13,0x01])))   # RXM-SFRBX @ 1 Hz
s.write(msg(0x06,0x09,bytes([0,0,0,0, 0xFF,0xFF,0,0, 0,0,0,0, 0x17])))  # CFG-CFG save
time.sleep(0.5); s.close()
print("raw messages enabled + saved to flash")
PY
sudo systemctl start str2str_tcp str2str_local_ntrip_caster
```

### 3.3 RTKBase settings (`/home/pi/rtkbase/settings.conf`)
Key values for this base — **watch the quoting** (see Troubleshooting):
```ini
[main]
position='LAT LON HEIGHT'                # base lat lon ellipsoidal-height
com_port='ttyACM0'                        # NO "/dev/" prefix
com_port_settings='115200:8:n:1'
receiver_format='ubx'                     # single quotes, value must be bare 'ubx'

[local_ntrip_caster]
local_ntripc_user=''                      # empty user+pwd => open (no auth)
local_ntripc_pwd=''
local_ntripc_port='2101'
local_ntripc_mnt_name='RTK-BASE'
```

**Base position accuracy.** The value above is a single-point fix (~1–2 m
absolute). RTK gives cm-level accuracy *relative to whatever the base claims*,
so for local work this is fine — but every rover is offset by the base's
absolute error. For true georeferencing, survey the antenna monument or average
a long survey-in and update `position=`.

### 3.4 Services (systemd, boot-persistent)
```bash
sudo systemctl enable --now str2str_tcp                 # receiver -> TCP:5015
sudo systemctl enable --now str2str_local_ntrip_caster  # TCP:5015 -> caster:2101
sudo systemctl enable --now rtkbase_web                 # web UI on :80
```
Data flow: `str2str_tcp` relays raw UBX from the receiver to `localhost:5015`;
the caster reads that, converts UBX→RTCM3 and serves mount `RTK-BASE` on port
2101.

### 3.5 Web UI
`http://192.168.1.10` (login required). Shows satellites, base position, map,
and per-service controls.

### 3.6 Verify the base
```bash
cd /home/pi/GPS-client
./ntrip_client.py --duration 15         # should show base pos + observations
```
Expect: `BASE POSITION` populated, MSM obs messages (1077/1087/…) at ~1 Hz,
`RTK READINESS ... READY`, CRC errors 0.

---

## 4. Rover setup

### 4.1 Receiver configuration
Configure the rover receiver (once, saved to its flash) to:
- **Accept RTCM3 input** on the port you'll feed (u-blox does this by default in
  RTK-rover mode).
- **Output NMEA GGA** (needed to read fix quality) — on by default for most
  u-blox; ensure GGA is enabled. `GST` too if you want an accuracy estimate.
- Set the **dynamic/platform model** to match use (stationary, pedestrian,
  automotive…). For an F9P, set the rover into RTK-rover navigation mode.
- 1 Hz update (or higher) at 115200 baud.

For a u-blox rover you can enable NMEA GGA + GST with the same CFG-MSG trick as
above (message class/id: GGA = 0xF0 0x00, GST = 0xF0 0x07), or use u-center.

### 4.2 Connect and run
Attach the rover to the machine (a second USB receiver shows up as e.g.
`/dev/ttyACM1` — **not** `/dev/ttyACM0`, which is the base). Then:

```bash
cd /home/pi/GPS-client
./ntrip_client.py --rover-port /dev/ttyACM1 --rover-baud 115200
```

The client:
1. Connects to the caster `RTK-BASE` mount.
2. Streams RTCM3 corrections **into** the rover's serial port.
3. Reads the rover's NMEA back and displays live fix quality.

For a networked/VRS caster that needs the rover's position upstream, add
`--send-gga`.

### 4.2.1 Stable device name for the rover (udev)
USB receivers can enumerate as `/dev/ttyACM1`, `/dev/ttyACM2`, … depending on
plug order, and the base is already `ttyACM0`. Give the rover a fixed symlink
(`/dev/gps-rover`) so the command never changes and you can't accidentally point
the client at the base.

First read the rover's USB identifiers (plug in **only** the rover, or compare
against the base):
```bash
udevadm info -a -n /dev/ttyACM1 | grep -E 'idVendor|idProduct|serial|KERNELS' | head
```

Create `/etc/udev/rules.d/99-gps-rover.rules`.

**Preferred — match by serial number** (works when the receiver reports a unique
`serial`, e.g. most ZED-F9P). This is robust regardless of which USB socket is
used:
```udev
# u-blox rover -> /dev/gps-rover   (replace serial with the rover's ATTRS{serial})
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{serial}=="DBADEADBEEF", \
  SYMLINK+="gps-rover", MODE="0660", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"
```

**Fallback — match by physical USB port** (use when the receiver has no unique
serial, e.g. the NEO-M8P which reports a generic id). The symlink then follows
whichever device is plugged into *that specific socket*, so always use the same
port:
```udev
# Whatever u-blox is in this USB path -> /dev/gps-rover
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", KERNELS=="1-1.2:1.0", \
  SYMLINK+="gps-rover", MODE="0660", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"
```
Find the `KERNELS==` value in the `udevadm info -a` output above (the first
`KERNELS` entry that looks like `1-1.2:1.0`).

`ENV{ID_MM_DEVICE_IGNORE}="1"` stops ModemManager from probing the receiver.
Apply and verify:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/gps-rover        # -> ../ttyACMx
```
Then run the client against the stable name:
```bash
./ntrip_client.py --rover-port /dev/gps-rover
```

### 4.3 Reading the fix status
| Status | Meaning | Accuracy |
|--------|---------|----------|
| `NO FIX` | No solution yet | — |
| `SINGLE (autonomous)` | No corrections applied | ~2–5 m |
| `DGPS` | Code-differential | ~0.5–2 m |
| `RTK FLOAT` | Carrier phase, ambiguities not fixed | ~10–50 cm |
| `RTK FIXED` | Ambiguities resolved — target state | ~1–2 cm |

Typical progression after starting: `SINGLE → FLOAT → FIXED`, from a few seconds
(F9P, good sky) up to a minute (M8P). Watch `CORR AGE` — corrections should be
**< ~5 s** old; if it climbs, the link is stalling.

### 4.4 Using a third-party rover instead
Any NTRIP rover app works — point it at:
```
Host: 192.168.1.10   Port: 2101   Mount: RTK-BASE   (no username/password)
```

---

## 5. `ntrip_client.py` reference

| Mode | Command | Purpose |
|------|---------|---------|
| Monitor | `./ntrip_client.py` | Decode/validate what the caster sends |
| Sourcetable | `./ntrip_client.py --sourcetable` | List mounts |
| Rover | `./ntrip_client.py --rover-port /dev/ttyACM1` | Feed a receiver, show fix |

Common options: `--host --port --mount -u -p --interval --duration
--rover-baud --send-gga`. Standard library only (rover mode also needs
`pyserial`).

---

## 6. Troubleshooting (issues actually hit on this build)

| Symptom | Cause | Fix |
|---------|-------|-----|
| `str2str_tcp` crash-loops, *"stream server start error"* | `com_port` had a `/dev/` prefix; this `str2str` build rejects absolute paths | Set `com_port='ttyACM0'` (no `/dev/`) |
| Web UI modal *"Main service is off or malfunctioning"* | `receiver_format="ubx"` stored with **double** quotes; web code strips only single quotes, so the format check fails | Use `receiver_format='ubx'` (single quotes / bare value) |
| Status page: satellites but **position 0,0,0** | No ephemeris — receiver wasn't sending UBX raw (`RXM-SFRBX`) | Enable RXM-RAWX + RXM-SFRBX, save to flash (§3.2) |
| Caster connects but rover **never fixes** | Mount was sending only base position (1005/1008/1033), **no observations** | Same raw-output fix (§3.2); verify with `ntrip_client.py` monitor |
| Web app logs look empty when debugging | Python stdout is block-buffered under systemd | Drop-in `Environment=PYTHONUNBUFFERED=1` on `rtkbase_web.service` |
| `CORR AGE` climbing / `LINK STALLED` | Network drop or base pipeline down | Check `systemctl status str2str_tcp str2str_local_ntrip_caster` |
| Rover shows `SINGLE`, never `FLOAT/FIXED` | Baseline too long, poor sky view, or too few common sats | Improve antenna siting; shorten baseline; use dual-band (F9P) |

Quick health checks:
```bash
systemctl is-active str2str_tcp str2str_local_ntrip_caster rtkbase_web
ss -ltnp | grep -E ':2101|:5015'          # caster + relay listening
cd /home/pi/GPS-client && ./ntrip_client.py --duration 10   # verify feed
```

---

## 7. Reference

**Ports:** `5015` receiver→caster relay (localhost) · `2101` NTRIP caster ·
`80` RTKBase web UI.

**RTCM3 messages on the mount:** `1005` base position · `1077/1087/1097/1107/1127`
GPS/GLONASS/Galileo/SBAS/BeiDou MSM7 observations · `1019/1020/…` ephemeris ·
`1008/1033` antenna/receiver descriptors · `1230` GLONASS biases.

**Base coordinates:** set to your surveyed antenna position in
`settings.conf` `position=` (decimal degrees + ellipsoidal height).

**Glossary:** *ARP* antenna reference point · *MSM* Multiple Signal Message
(modern RTCM obs) · *NTRIP* Networked Transport of RTCM via Internet Protocol ·
*RTCM3* the correction message standard · *baseline* base↔rover distance.

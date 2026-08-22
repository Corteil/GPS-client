# RTK GNSS Base + Rover — Fresh Raspberry Pi OS Install

Build the whole system from a blank SD card: a u-blox **base station** that
streams RTCM3 corrections over a local **NTRIP caster**, plus the tools to drive
a **rover** to a centimetre-level RTK fix.

This guide assumes nothing is installed yet. For the concepts, data-flow diagram
and fix-status reference, see **SETUP-GUIDE.md**.

---

## 0. What you need

**Hardware**
- Raspberry Pi 4 or 5 (or CM4/CM5 carrier), 2 GB+ RAM, quality power supply
- microSD card (16 GB+) or NVMe/USB SSD
- u-blox GNSS receiver for the **base** — NEO-M8P / NEO-M8T / ZED-**F9P**
  (F9P is easiest; must be able to output raw `RXM-RAWX`/`RXM-SFRBX`)
- Multi-GNSS **active antenna** on a rigid, sky-open mount (base)
- For the **rover**: a second u-blox receiver (ZED-F9P recommended) + antenna

**Software**
- Raspberry Pi Imager (on your laptop) to flash the OS
- Network access on the Pi (Ethernet or Wi-Fi)

**Conventions**
- Commands run as user `pi` unless prefixed with `sudo`.
- The base receiver appears here as `/dev/ttyACM0` (USB). UART receivers use
  `/dev/ttyAMA0` / `/dev/serial0` — substitute accordingly.

---

## 1. Flash Raspberry Pi OS

1. Install **Raspberry Pi Imager** on your computer.
2. Choose **Raspberry Pi OS (64-bit)** — the *Lite* image is fine (this is a
   headless server; the web UI is browser-based).
3. Click the gear / **Edit Settings** before writing and set:
   - Hostname (e.g. `rtkbase`)
   - Enable **SSH** (password or key)
   - Username `pi` + password
   - Wi-Fi country + credentials (if not using Ethernet)
   - Locale / timezone
4. Write the card, boot the Pi, and SSH in:
   ```bash
   ssh pi@rtkbase.local        # or the Pi's IP
   ```

---

## 2. Update the OS and give the Pi a stable IP

```bash
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

Reserve a **static / DHCP-reserved IP** for the Pi (in your router, by MAC) so
rovers always find the caster at the same address. Note that address — it is
referred to below as `PI_IP`.

Optional prerequisites used later:
```bash
sudo apt install -y git python3-serial socat
```

---

## 3. Connect the receiver and find its port

Plug the **base** GNSS receiver into USB, then:
```bash
ls -l /dev/serial/by-id/     # stable name for the receiver
dmesg | grep -iE 'ttyACM|u-blox|cdc_acm' | tail
```
You should see something like `.../u-blox_GNSS_receiver-if00 -> ../../ttyACM0`.
Record the port (e.g. `ttyACM0`) — you'll need the **bare name without
`/dev/`** later.

Quick sanity check that it's talking (raw NMEA):
```bash
timeout 3 cat /dev/ttyACM0        # expect $GNGGA/$GNTXT lines (Ctrl-C to stop)
```

---

## 4. Install RTKBase

RTKBase bundles RTKLIB (`str2str`/`rtkrcv`), a web UI, and all systemd services.
Run the official installer:

```bash
cd ~
wget https://raw.githubusercontent.com/Stefal/rtkbase/master/tools/install.sh -O install.sh
chmod +x install.sh
sudo ./install.sh --all release
```

This installs dependencies, builds RTKLIB (rtklibexplorer fork), unpacks RTKBase
to `~/rtkbase`, installs the systemd units, and sets up gpsd/chrony. It takes a
while on first run.

> If you prefer to do it in stages, the same steps are available individually:
> `--dependencies`, `--rtklib`, `--rtkbase-release`, `--unit-files`,
> `--gpsd-chrony`, `--detect-gnss`, `--start-services`.

Let the installer detect the receiver:
```bash
cd ~/rtkbase
sudo ./tools/install.sh --detect-gnss        # writes com_port into settings.conf
```

---

## 5. Configure the receiver for RAW output — the critical step

The pipeline needs the receiver to emit **UBX raw** messages so it can build a
*complete* RTCM stream (observations **and** ephemeris) and so the web UI can
compute a live position:

- `RXM-RAWX`  (0x02 0x15) — raw carrier-phase / pseudorange observations
- `RXM-SFRBX` (0x02 0x13) — broadcast navigation frames (ephemeris)

**ZED-F9P / Mosaic / Unicore:** the installer can do this for you:
```bash
sudo ./tools/install.sh --detect-gnss --configure-gnss
```

**NEO-M8P / M8T (installer has no auto-profile):** enable the raw messages
manually and save them to the receiver's flash so they survive a power cycle.
Stop the services first to free the serial port:
```bash
sudo systemctl stop str2str_tcp str2str_local_ntrip_caster 2>/dev/null
python3 - <<'PY'
import serial, time
def ck(p):
    a=b=0
    for x in p: a=(a+x)&0xFF; b=(b+a)&0xFF
    return bytes([a,b])
def msg(cls,mid,pl):
    body=bytes([cls,mid,len(pl)&0xFF,(len(pl)>>8)&0xFF])+pl
    return b'\xb5\x62'+body+ck(body)
port='/dev/ttyACM0'                              # <-- your receiver port
s=serial.Serial(port,115200,timeout=1)
s.write(msg(0x06,0x01,bytes([0x02,0x15,0x01])))  # RXM-RAWX  @ 1 Hz
s.write(msg(0x06,0x01,bytes([0x02,0x13,0x01])))  # RXM-SFRBX @ 1 Hz
s.write(msg(0x06,0x09,bytes([0,0,0,0, 0xFF,0xFF,0,0, 0,0,0,0, 0x17])))  # CFG-CFG save
time.sleep(0.5); s.close()
print("raw messages enabled and saved to flash")
PY
```
(If your M8P is on UART not USB, use the correct port/baud.)

---

## 6. Configure RTKBase (`~/rtkbase/settings.conf`)

Edit the key values. **Two quoting rules that will silently break things if you
get them wrong** (learned the hard way — see Troubleshooting):

```ini
[main]
position='LAT LON HEIGHT'          # set in step 7 (lat lon ellipsoidal-height)
com_port='ttyACM0'                 # bare device name, NO "/dev/" prefix
com_port_settings='115200:8:n:1'
receiver_format='ubx'              # single quotes, value must be bare 'ubx'

[local_ntrip_caster]
local_ntripc_user=''               # empty user + pwd  => open caster (no auth)
local_ntripc_pwd=''
local_ntripc_port='2101'
local_ntripc_mnt_name='RTK-BASE'   # the mountpoint rovers will request
```

- `com_port` must **not** include `/dev/` — this `str2str` build rejects
  absolute paths and the main service will crash-loop.
- `receiver_format` must be the bare word `ubx` in **single** quotes. A
  double-quoted `"ubx"` makes the web UI think the service is down.

---

## 7. Set the base position

RTK is only as good as the base coordinate you declare. Two options:

**A. Known / surveyed point (best).** Enter the antenna reference point you
already know (from a survey monument, OS benchmark, PPP post-processing, etc.):
```ini
position='LAT LON HEIGHT'          # decimal degrees + ellipsoidal height (m)
```

**B. Survey-in / autonomous average (quick start).** Let the receiver run, then
take its averaged single-point position. After services are up (step 8) you can
read a live position from the web UI's status page or from the receiver, then
paste it into `position=`. This is ~1–2 m absolute — fine for local work
(relative rover accuracy stays cm-level), but every rover is offset by that same
error, so re-survey later if you need true georeferencing.

Height is **ellipsoidal** (WGS84), not mean-sea-level.

---

## 8. Start and enable the services

```bash
cd ~/rtkbase
sudo ./tools/install.sh --start-services        # or enable them directly:
sudo systemctl enable --now str2str_tcp                 # receiver -> TCP:5015
sudo systemctl enable --now str2str_local_ntrip_caster  # TCP:5015 -> caster:2101
sudo systemctl enable --now rtkbase_web                 # web UI on :80
```

Make the web app log unbuffered (so its diagnostics are readable in `journalctl`
— they are block-buffered by default):
```bash
sudo mkdir -p /etc/systemd/system/rtkbase_web.service.d
printf '[Service]\nEnvironment=PYTHONUNBUFFERED=1\n' | \
  sudo tee /etc/systemd/system/rtkbase_web.service.d/unbuffered.conf
sudo systemctl daemon-reload && sudo systemctl restart rtkbase_web
```

---

## 9. Web UI

Open `http://PI_IP` (or `http://rtkbase.local`). Set a web password on first
use. The status page shows satellites, base position, a map, and per-service
controls.

---

## 10. Install the client / diagnostic tools

Copy this `GPS-client` folder onto the Pi (or `git`/`scp` it):
```bash
# from this repo/folder
mkdir -p ~/GPS-client
cp ntrip_client.py README.md SETUP-GUIDE.md FRESH-INSTALL.md ~/GPS-client/
chmod +x ~/GPS-client/ntrip_client.py
sudo apt install -y python3-serial        # needed for rover mode
```

Verify the base is producing a full correction stream:
```bash
cd ~/GPS-client
./ntrip_client.py --host 127.0.0.1 --duration 15
```
Expect: `BASE POSITION` populated, MSM observation messages (1077/1087/…) at
~1 Hz, ephemeris appearing intermittently, `RTK READINESS ... READY`, and
`CRC errors: 0`.

---

## 11. Rover setup

1. **Configure the rover receiver** (once, saved to its flash): accept RTCM3
   input on its port (u-blox default in RTK-rover mode), output **NMEA GGA**
   (and `GST` for an accuracy figure), set the platform/dynamic model, 1 Hz.

2. **Give it a stable device name** so it never collides with the base. Create
   `/etc/udev/rules.d/99-gps-rover.rules`:
   ```udev
   # Preferred: match by serial (e.g. ZED-F9P). Get it from:
   #   udevadm info -a -n /dev/ttyACM1 | grep -E 'idVendor|serial|KERNELS'
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{serial}=="YOUR_SERIAL", \
     SYMLINK+="gps-rover", MODE="0660", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"

   # Fallback (no unique serial, e.g. M8P): match the physical USB socket.
   #SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", KERNELS=="1-1.2:1.0", \
   #  SYMLINK+="gps-rover", MODE="0660", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"
   ```
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ls -l /dev/gps-rover
   ```

3. **Feed corrections and watch the fix** (from the Pi or any machine on the
   network):
   ```bash
   ./ntrip_client.py --host PI_IP --mount RTK-BASE --rover-port /dev/gps-rover
   ```
   Progression: `SINGLE → RTK FLOAT → RTK FIXED`. Keep `CORR AGE` under ~5 s.

   Any third-party NTRIP rover app works too — point it at
   `Host: PI_IP  Port: 2101  Mount: RTK-BASE` (no username/password).

---

## 12. Post-install verification checklist

```bash
systemctl is-active str2str_tcp str2str_local_ntrip_caster rtkbase_web   # all "active"
systemctl is-enabled str2str_tcp str2str_local_ntrip_caster rtkbase_web  # all "enabled"
ss -ltnp | grep -E ':2101|:5015|:80 '        # caster + relay + web listening
cd ~/GPS-client && ./ntrip_client.py --host 127.0.0.1 --duration 10       # READY, obs flowing
```
Reboot once and re-run the checklist to confirm everything comes back on its own
(receiver raw config is in flash, services are enabled).

---

## 13. Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `str2str_tcp` crash-loops, *"stream server start error"* | `com_port` has a `/dev/` prefix | Set `com_port='ttyACM0'` (bare name) |
| Web UI modal *"Main service is off or malfunctioning"* | `receiver_format="ubx"` (double-quoted); web code strips only single quotes | Use `receiver_format='ubx'` |
| Satellites show but **position is 0,0,0** | No ephemeris — receiver not sending `RXM-SFRBX` | Enable RXM-RAWX + RXM-SFRBX, save to flash (step 5) |
| Caster connects but rover **never fixes**; mount has only 1005/1008/1033 | No observations in the stream (raw not enabled) | Same raw-output fix (step 5); verify with `ntrip_client.py` |
| Web app logs look empty while debugging | Python stdout block-buffered under systemd | `PYTHONUNBUFFERED=1` drop-in (step 8) |
| Rover stuck at `SINGLE` | Long baseline, poor sky view, few common sats | Better antenna siting; shorten baseline; use dual-band F9P |
| `CORR AGE` climbing / `LINK STALLED` | Network drop or base pipeline down | `systemctl status str2str_tcp str2str_local_ntrip_caster` |
| Receiver port keeps changing (`ttyACM1`↔`ttyACM2`) | USB enumeration order | Add the udev rule (step 11.2) |

---

## 14. Ports & data flow recap

```
receiver (USB, raw UBX) --> str2str_tcp --> TCP:5015 (localhost)
    --> str2str_local_ntrip_caster --> NTRIP caster tcp/2101  mount "RTK-BASE"
    --> rover (RTCM3 in, NMEA out) --> RTK fix
```
`5015` internal relay · `2101` NTRIP caster · `80` web UI.
```

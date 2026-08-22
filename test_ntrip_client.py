"""Unit tests for the pure (non-network) parts of ntrip_client."""
import unittest

import ntrip_client as nc


def with_checksum(sentence_no_star):
    """Append a correct NMEA checksum to a '$...' body (no '*')."""
    body = sentence_no_star[1:]
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{sentence_no_star}*{cs:02X}"


# A genuine RTCM3 type-1077 frame captured from the caster (header+payload+CRC).
REAL_RTCM_1077 = bytes.fromhex(
    "d300a84350008ee24eba0000001c206d80000000200000007fe88888a90748676747a8e0"
    "000000001e172a22cfbf19b47f98062b6604a03440cb78ce059f98ffa3fd53ee9032c020"
    "29a90800a525f448bd78662c36da3cebcb0cc1778c522b200eeaa094bdc01e80e153273f6"
    "7d9c07c633ebf9a1f1a94204e6abf006a722489223cb2244922489222b800501584412856"
    "130521585a0b87025e33ff5dfd14732bf3bfd42099c03ab23e008f5956"
)


class CRC24Q(unittest.TestCase):
    def test_real_frame_crc_matches(self):
        # For a valid RTCM3 frame, CRC-24Q over header+payload equals the
        # trailing 3 CRC bytes. This validates the actual RTCM use case.
        length = ((REAL_RTCM_1077[1] & 0x03) << 8) | REAL_RTCM_1077[2]
        crc_rx = (REAL_RTCM_1077[-3] << 16) | (REAL_RTCM_1077[-2] << 8) | REAL_RTCM_1077[-1]
        self.assertEqual(nc.crc24q(REAL_RTCM_1077[:3 + length]), crc_rx)

    def test_detects_corruption(self):
        bad = bytearray(REAL_RTCM_1077)
        bad[10] ^= 0xFF  # flip a payload byte
        length = ((bad[1] & 0x03) << 8) | bad[2]
        crc_rx = (bad[-3] << 16) | (bad[-2] << 8) | bad[-1]
        self.assertNotEqual(nc.crc24q(bytes(bad[:3 + length])), crc_rx)

    def test_empty(self):
        self.assertEqual(nc.crc24q(b""), 0x000000)


class Ecef(unittest.TestCase):
    def test_equator_prime_meridian(self):
        lat, lon, h = nc.ecef_to_llh(6378137.0, 0.0, 0.0)
        self.assertAlmostEqual(lat, 0.0, places=6)
        self.assertAlmostEqual(lon, 0.0, places=6)
        self.assertAlmostEqual(h, 0.0, places=3)

    def test_typical_surface_point(self):
        # ~45 deg N on the ellipsoid surface, round-tripped from a known ECEF.
        lat, lon, h = nc.ecef_to_llh(4517590.9, 0.0, 4487348.4)
        self.assertAlmostEqual(lat, 45.0, places=1)
        self.assertAlmostEqual(lon, 0.0, places=6)
        self.assertLess(abs(h), 50.0)


class Nmea(unittest.TestCase):
    def test_checksum_ok(self):
        line = with_checksum("$GNGGA,120000.00,5100.00000,N,00000.00000,E,4,18,0.6,45.0,M,45.4,M,1.2,0000")
        self.assertTrue(nc.nmea_checksum_ok(line))

    def test_checksum_bad(self):
        line = with_checksum("$GNGGA,120000.00,5100.00000,N,00000.00000,E,4,18,0.6,45.0,M,45.4,M,1.2,0000")
        tampered = line[:-2] + ("00" if line[-2:] != "00" else "11")
        self.assertFalse(nc.nmea_checksum_ok(tampered))

    def test_parse_gga_rtk_fixed(self):
        line = with_checksum("$GNGGA,120000.00,5100.00000,N,00030.00000,E,4,18,0.65,45.0,M,45.4,M,1.2,0000")
        g = nc.parse_gga(line)
        self.assertEqual(g["quality"], 4)
        self.assertEqual(nc.FIX_QUALITY[g["quality"]], "RTK FIXED")
        self.assertEqual(g["sats"], 18)
        self.assertAlmostEqual(g["lat"], 51.0, places=6)      # 51 deg 00.00000'
        self.assertAlmostEqual(g["lon"], 0.5, places=6)       # 000 deg 30.00000'
        self.assertAlmostEqual(g["corr_age"], 1.2, places=3)

    def test_parse_gga_south_west_signs(self):
        line = with_checksum("$GNGGA,120000.00,5100.00000,S,00030.00000,W,5,12,0.9,45.0,M,45.4,M,2.0,0000")
        g = nc.parse_gga(line)
        self.assertEqual(g["quality"], 5)
        self.assertLess(g["lat"], 0)
        self.assertLess(g["lon"], 0)

    def test_parse_gga_rejects_non_gga(self):
        self.assertIsNone(nc.parse_gga("$GNRMC,120000,A,5100.0,N,00030.0,E,0,0,010101,,*00"))

    def test_parse_gst(self):
        line = with_checksum("$GNGST,120000.00,12.3,0.9,0.7,45.0,0.012,0.010,0.020")
        hstd = nc.parse_gst(line)
        self.assertAlmostEqual(hstd, (0.012 ** 2 + 0.010 ** 2) ** 0.5, places=6)


class MsgNames(unittest.TestCase):
    def test_constellation_ranges(self):
        self.assertEqual(nc.constellation_of(1077), "GPS")
        self.assertEqual(nc.constellation_of(1087), "GLONASS")
        self.assertEqual(nc.constellation_of(1127), "BeiDou")
        self.assertIsNone(nc.constellation_of(1005))

    def test_msg_name_known_and_unknown(self):
        self.assertEqual(nc.msg_name(1005), "Base ARP position (ECEF)")
        self.assertTrue(nc.msg_name(9999).startswith("RTCM type"))


if __name__ == "__main__":
    unittest.main()

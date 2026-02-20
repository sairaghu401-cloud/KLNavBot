"""
KLNavBot — gps_module.py
NEO-6M GPS Parser — MicroPython

YOUR GPS Wiring:
  Module  : NEO-6M
  UART    : UART1
  TX pin  : GPIO 17  (ESP32 TX → GPS RX)
  RX pin  : GPIO 16  (ESP32 RX ← GPS TX)
  Baud    : 9600

Parses NMEA sentences $GPGGA / $GNGGA and converts
raw DDDMM.MMMM format to decimal degrees for Firebase.

Author : KLNavBot Team
Version: 2.0.0
"""

from machine import UART


# ═══════════════════════════════════════════════════════════
# GPS PIN DEFINITIONS
# ═══════════════════════════════════════════════════════════

_UART_ID   = 1      # UART1
_TX_PIN    = 17     # ESP32 TX → GPS RX
_RX_PIN    = 16     # ESP32 RX ← GPS TX
_BAUD_RATE = 9600   # standard NEO-6M baud rate


# ═══════════════════════════════════════════════════════════
# GPS MODULE CLASS
# ═══════════════════════════════════════════════════════════

class GPSModule:
    """
    Reads NMEA sentences from a NEO-6M GPS over UART1
    and returns clean decimal lat/lon values.

    Example:
        from gps_module import GPSModule
        gps = GPSModule()
        lat, lon, sats = gps.read()
        if lat:
            print(lat, lon)
    """

    def __init__(self):
        """Initialise UART1 for GPS communication."""
        self._uart = UART(
            _UART_ID,
            baudrate=_BAUD_RATE,
            tx=_TX_PIN,
            rx=_RX_PIN
        )
        self.last_lat  = None   # last valid latitude
        self.last_lon  = None   # last valid longitude
        self.last_sats = 0      # last satellite count
        self.fix       = False  # True once first fix received

        print(f"[GPS] UART{_UART_ID} ready  —  TX={_TX_PIN}  RX={_RX_PIN}  @ {_BAUD_RATE} baud")
        print("[GPS] Waiting for satellite fix ...")

    # ────────────────────────────────────────────────────────
    # Private helpers
    # ────────────────────────────────────────────────────────

    @staticmethod
    def _nmea_to_decimal(raw: str, direction: str):
        """
        Convert NMEA DDDMM.MMMM to decimal degrees.

        Formula:
            decimal = degrees + (minutes / 60)
            Negate for South (S) or West (W).

        Returns float or None if raw string is empty.
        """
        if not raw or raw == "":
            return None

        try:
            value   = float(raw)
            degrees = int(value / 100)           # integer degrees part
            minutes = value - (degrees * 100)    # remaining minutes part
            decimal = degrees + (minutes / 60.0)

            if direction in ('S', 'W'):
                decimal = -decimal

            return round(decimal, 6)             # 6 decimal places ≈ 11 cm accuracy
        except (ValueError, TypeError):
            return None

    @staticmethod
    def _parse_gga(line: str):
        """
        Parse a $GPGGA or $GNGGA NMEA sentence.

        GGA sentence format:
          $GPGGA,HHMMSS.ss,LLLL.LL,a,YYYYY.YY,a,q,NN,D.D,H.H,M,...
          Index:  0         1        2 3        4 5  6 7

          [6] Fix quality : 0 = no fix, 1 = GPS fix, 2 = DGPS fix
          [7] Satellites  : number of satellites in use

        Returns (lat, lon, satellites) or (None, None, None).
        """
        # Only process GGA sentences
        if "$GPGGA" not in line and "$GNGGA" not in line:
            return None, None, None

        parts = line.split(",")

        # Need at least 8 fields for a valid GGA sentence
        if len(parts) < 8:
            return None, None, None

        # Check fix quality — 0 means no satellite lock yet
        fix_quality = parts[6].strip()
        if fix_quality == "" or fix_quality == "0":
            return None, None, None

        try:
            lat  = GPSModule._nmea_to_decimal(parts[2], parts[3])
            lon  = GPSModule._nmea_to_decimal(parts[4], parts[5])
            sats = parts[7].strip()
            return lat, lon, sats
        except Exception:
            return None, None, None

    # ────────────────────────────────────────────────────────
    # Public API
    # ────────────────────────────────────────────────────────

    def read(self):
        """
        Read one line from UART and parse it.

        Returns:
            (lat: float, lon: float, satellites: str)  — on valid fix
            (None, None, None)                          — no data / no fix yet
        """
        # Nothing available from GPS module yet
        if not self._uart.any():
            return None, None, None

        raw = self._uart.readline()
        if not raw:
            return None, None, None

        # Decode bytes → string safely
        try:
            line = raw.decode("utf-8").strip()
        except (UnicodeDecodeError, Exception):
            return None, None, None

        lat, lon, sats = self._parse_gga(line)

        # Update cached values on valid fix
        if lat is not None and lon is not None:
            self.last_lat  = lat
            self.last_lon  = lon
            self.last_sats = sats
            self.fix       = True

            # First fix message
            print(f"[GPS] FIX ACQUIRED  lat={lat}  lon={lon}  sats={sats}")
            print("      " + "-" * 40)

        return lat, lon, sats

    def last_known(self):
        """
        Return the most recent valid GPS fix (cached).
        Useful when current read returns None.

        Returns:
            (lat, lon, sats) — last valid values
            (None, None, 0)  — if no fix has been received yet
        """
        return self.last_lat, self.last_lon, self.last_sats

    def has_fix(self) -> bool:
        """Return True if at least one valid GPS fix has been received."""
        return self.fix

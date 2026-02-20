"""
KLNavBot — main.py (MicroPython for Thonny)
ESP32 firmware using 3-file structure:
  ├── gps_module.py      GPS parsing (NEO-6M, UART1)
  ├── motor_control.py   Motor driving (L298N)
  └── main.py            This file — WiFi, Firebase, main loop

YOUR Hardware:
  WiFi          : Replace WIFI_SSID / WIFI_PASSWORD below
  Firebase      : Replace FIREBASE_URL below
  GPS (NEO-6M)  : UART1  TX=17  RX=16  @ 9600 baud
  Ultrasonic    : TRIG=19  ECHO=18
  Motor A (Left): IN1=25  IN2=26  ENA=27
  Motor B (Right): IN3=32  IN4=33  ENB=14
  Obstacle      : stops if distance < 20 cm
  Firebase poll : every 500 ms

Upload order in Thonny (File → Save copy → MicroPython device):
  1. motor_control.py
  2. gps_module.py
  3. main.py
  Then press EN (reset) on ESP32.

Author : KLNavBot Team
Version: 2.0.0
"""

import gc
import time
import network
import urequests
import ujson
from machine import Pin, time_pulse_us

# ── Import your own modules ──────────────────────────────────
from motor_control import MotorController
from gps_module    import GPSModule


# ═══════════════════════════════════════════════════════════
# SECTION 1 — CONFIGURATION
# ═══════════════════════════════════════════════════════════

# ── WiFi credentials ─────────────────────────────────────────
WIFI_SSID     = "YOUR_WIFI_SSID"      # 🔒 Replace with your WiFi name
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"  # 🔒 Replace with your WiFi password

# ── Firebase Realtime Database ────────────────────────────────
FIREBASE_URL  = "https://YOUR_PROJECT_ID-default-rtdb.firebaseio.com"  # 🔒 Replace
FIREBASE_AUTH = ""   # 🔒 Optional: your database secret (leave "" for open rules)

# ── Ultrasonic sensor pins ────────────────────────────────────
TRIG_PIN = 19   # HC-SR04 trigger
ECHO_PIN = 18   # HC-SR04 echo

# ── Obstacle stop distance ────────────────────────────────────
OBSTACLE_THRESHOLD_CM = 20   # halt motors if object closer than this

# ── Motor default speed (0–1023) ─────────────────────────────
MOTOR_SPEED = 700

# ── Timing intervals (milliseconds) ──────────────────────────
FIREBASE_POLL_MS  = 500    # how often to check for new commands
GPS_PUSH_MS       = 2000   # how often to push GPS to Firebase
STATUS_PUSH_MS    = 1000   # how often to push heartbeat status


# ═══════════════════════════════════════════════════════════
# SECTION 2 — WIFI
# ═══════════════════════════════════════════════════════════

def connect_wifi() -> network.WLAN:
    """
    Connect to WiFi access point.
    Retries up to 30 times, restarts ESP32 on failure.
    """
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    # Skip if already connected (e.g. after soft reset)
    if wlan.isconnected():
        print("[WiFi] Already connected:", wlan.ifconfig()[0])
        return wlan

    print(f"[WiFi] Connecting to '{WIFI_SSID}'", end="")
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)

    attempts = 0
    while not wlan.isconnected() and attempts < 30:
        time.sleep(1)
        print(".", end="")
        attempts += 1

    if wlan.isconnected():
        print(f"\n[WiFi] ✅ Connected!")
        print(f"       IP      : {wlan.ifconfig()[0]}")
        print(f"       Gateway : {wlan.ifconfig()[2]}")
    else:
        print("\n[WiFi] ❌ Failed — restarting in 5s ...")
        time.sleep(5)
        import machine
        machine.reset()

    return wlan


# ═══════════════════════════════════════════════════════════
# SECTION 3 — FIREBASE REST HELPERS
# ═══════════════════════════════════════════════════════════

def _url(path: str) -> str:
    """Build full Firebase REST endpoint URL."""
    base = f"{FIREBASE_URL}{path}.json"
    return base + f"?auth={FIREBASE_AUTH}" if FIREBASE_AUTH else base


def firebase_get(path: str):
    """
    Read a Firebase node (HTTP GET).
    Returns parsed dict/value, or None on error.
    """
    try:
        r = urequests.get(_url(path), timeout=5)
        data = r.json()
        r.close()
        gc.collect()   # free RAM after HTTP response
        return data
    except Exception as e:
        print(f"[Firebase GET] {path} — {e}")
        return None


def firebase_put(path: str, data: dict) -> bool:
    """
    Overwrite a Firebase node (HTTP PUT).
    Use for initial setup of full nodes.
    """
    try:
        r = urequests.put(
            _url(path),
            data=ujson.dumps(data),
            headers={"Content-Type": "application/json"},
            timeout=5
        )
        r.close()
        gc.collect()
        return True
    except Exception as e:
        print(f"[Firebase PUT] {path} — {e}")
        return False


def firebase_patch(path: str, data: dict) -> bool:
    """
    Update specific fields in a Firebase node (HTTP PATCH).
    Use for partial updates like status and GPS.
    """
    try:
        r = urequests.patch(
            _url(path),
            data=ujson.dumps(data),
            headers={"Content-Type": "application/json"},
            timeout=5
        )
        r.close()
        gc.collect()
        return True
    except Exception as e:
        print(f"[Firebase PATCH] {path} — {e}")
        return False


# ═══════════════════════════════════════════════════════════
# SECTION 4 — ULTRASONIC SENSOR
# ═══════════════════════════════════════════════════════════

def setup_ultrasonic():
    """
    Initialise HC-SR04 pins.
    Returns (trig, echo) Pin objects.
    """
    trig = Pin(TRIG_PIN, Pin.OUT)
    echo = Pin(ECHO_PIN, Pin.IN)
    trig.off()
    print(f"[SENSOR] HC-SR04 ready  —  TRIG={TRIG_PIN}  ECHO={ECHO_PIN}")
    return trig, echo


def get_distance_cm(trig: Pin, echo: Pin):
    """
    Measure distance using HC-SR04 ultrasonic sensor.

    Steps:
      1. Send a 10 µs HIGH pulse on TRIG
      2. Measure duration of ECHO HIGH pulse
      3. Distance (cm) = duration × speed_of_sound / 2

    Returns float (cm), or None if no echo received (open path).
    """
    # Ensure trigger is LOW before pulse
    trig.off()
    time.sleep_us(2)

    # Send 10 µs trigger pulse
    trig.on()
    time.sleep_us(10)
    trig.off()

    # Measure echo — timeout 30 000 µs ≈ 5 m max range
    pulse = time_pulse_us(echo, 1, 30000)

    if pulse <= 0:
        return None   # no echo = clear path

    return (pulse * 0.0343) / 2.0


# ═══════════════════════════════════════════════════════════
# SECTION 5 — FIREBASE NODE INIT
# ═══════════════════════════════════════════════════════════

def init_firebase_nodes():
    """
    Write safe default values to Firebase on startup.
    Ensures the dashboard never reads stale data.
    """
    print("[Firebase] Initialising nodes ...")

    firebase_put("/robotStatus", {
        "obstacle"  : False,
        "state"     : "stopped",
        "lastUpdate": time.time()
    })

    firebase_put("/robotCommands", {
        "move"      : "stop",
        "timestamp" : time.time()
    })

    firebase_put("/robotGPS", {
        "lat"        : None,
        "lon"        : None,
        "satellites" : 0,
        "lastUpdate" : time.time()
    })

    print("[Firebase] ✅ Nodes initialised")


# ═══════════════════════════════════════════════════════════
# SECTION 6 — COMMAND DISPATCHER
# ═══════════════════════════════════════════════════════════

VALID_COMMANDS = {"forward", "backward", "left", "right", "stop"}

def execute_command(motors: MotorController, command: str):
    """
    Map a Firebase command string → motor action.
    Falls back to stop() for any unknown command.
    """
    if   command == "forward":  motors.forward(MOTOR_SPEED)
    elif command == "backward": motors.backward(MOTOR_SPEED)
    elif command == "left":     motors.left(MOTOR_SPEED)
    elif command == "right":    motors.right(MOTOR_SPEED)
    elif command == "stop":     motors.stop()
    else:
        print(f"[CMD] Unknown command '{command}' — defaulting to stop")
        motors.stop()


# ═══════════════════════════════════════════════════════════
# SECTION 7 — MAIN APPLICATION LOOP
# ═══════════════════════════════════════════════════════════

def main():
    print("\n" + "=" * 42)
    print("   KLNavBot v2.0.0  —  MicroPython")
    print("=" * 42 + "\n")

    # ── Hardware init ─────────────────────────────────────────
    trig, echo = setup_ultrasonic()          # HC-SR04
    motors     = MotorController(MOTOR_SPEED) # L298N via motor_control.py
    gps        = GPSModule()                  # NEO-6M via gps_module.py

    # ── Network + Firebase ────────────────────────────────────
    connect_wifi()
    init_firebase_nodes()

    # ── Loop state variables ──────────────────────────────────
    last_command        = "stop"   # most recent executed command
    motor_halted        = False    # True while obstacle is blocking
    last_firebase_ms    = 0        # timestamp of last Firebase poll
    last_gps_push_ms    = 0        # timestamp of last GPS push
    last_status_push_ms = 0        # timestamp of last status heartbeat

    print("\n✅ All systems ready — entering main loop\n")

    # ─────────────────────────────────────────────────────────
    while True:
        now = time.ticks_ms()
        gc.collect()   # prevent memory fragmentation over time

        # ╔══════════════════════════════════════════════════╗
        # ║  STEP 1 — Obstacle check (runs every iteration) ║
        # ╚══════════════════════════════════════════════════╝
        distance     = get_distance_cm(trig, echo)
        obstacle_now = (distance is not None and distance < OBSTACLE_THRESHOLD_CM)

        if obstacle_now:
            # ── New obstacle: halt once and report ───────────
            if not motor_halted:
                print(f"[SENSOR] 🛑 Obstacle {distance:.1f} cm — halting!")
                motors.stop()
                motor_halted = True
                firebase_patch("/robotStatus", {
                    "obstacle"  : True,
                    "state"     : "stopped",
                    "lastUpdate": time.time()
                })
            time.sleep(0.5)
            continue   # skip commands and GPS while blocked

        else:
            # ── Obstacle cleared: re-enable command handling ─
            if motor_halted:
                print("[SENSOR] 🚗 Path clear — ready for commands")
                motor_halted = False
                firebase_patch("/robotStatus", {
                    "obstacle"  : False,
                    "state"     : "stopped",
                    "lastUpdate": time.time()
                })

        # ╔══════════════════════════════════════════════════╗
        # ║  STEP 2 — Firebase command poll  (every 500 ms) ║
        # ╚══════════════════════════════════════════════════╝
        if time.ticks_diff(now, last_firebase_ms) >= FIREBASE_POLL_MS:
            last_firebase_ms = now

            data = firebase_get("/robotCommands")

            if data and isinstance(data, dict):
                new_cmd = data.get("move", "stop")

                # Reject invalid command strings
                if new_cmd not in VALID_COMMANDS:
                    new_cmd = "stop"

                # Only act if command actually changed
                if new_cmd != last_command:
                    last_command = new_cmd
                    print(f"[CMD] ▶ {new_cmd}")
                    execute_command(motors, new_cmd)

                    # Report new state back to dashboard
                    state = "stopped" if new_cmd == "stop" else "moving"
                    firebase_patch("/robotStatus", {
                        "state"     : state,
                        "lastUpdate": time.time()
                    })

        # ╔══════════════════════════════════════════════════╗
        # ║  STEP 3 — Status heartbeat push  (every 1 sec)  ║
        # ╚══════════════════════════════════════════════════╝
        if time.ticks_diff(now, last_status_push_ms) >= STATUS_PUSH_MS:
            last_status_push_ms = now
            state = "stopped" if last_command == "stop" else "moving"
            firebase_patch("/robotStatus", {
                "obstacle"  : False,
                "state"     : state,
                "lastUpdate": time.time()
            })

        # ╔══════════════════════════════════════════════════╗
        # ║  STEP 4 — GPS read & push        (every 2 sec)  ║
        # ╚══════════════════════════════════════════════════╝
        lat, lon, sats = gps.read()

        if lat is not None and lon is not None:
            if time.ticks_diff(now, last_gps_push_ms) >= GPS_PUSH_MS:
                last_gps_push_ms = now
                firebase_patch("/robotGPS", {
                    "lat"        : lat,
                    "lon"        : lon,
                    "satellites" : sats,
                    "lastUpdate" : time.time()
                })

        # Yield to prevent watchdog timeout
        time.sleep(0.2)


# ═══════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════
if __name__ == "__main__":
    main()

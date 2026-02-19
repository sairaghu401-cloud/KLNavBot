import network
import time
from machine import UART, Pin, time_pulse_us, PWM
import urequests
import ujson

# ================= WIFI =================
WIFI_SSID = "vivo"
WIFI_PASSWORD = "1234567890"

wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(WIFI_SSID, WIFI_PASSWORD)

print("Connecting to WiFi", end="")
while not wifi.isconnected():
    print(".", end="")
    time.sleep(1)

print("\n✅ WiFi Connected!")
print("IP Address:", wifi.ifconfig()[0])

# ================= FIREBASE CONFIG =================
FIREBASE_URL = "https://klu-robot-tracker-default-rtdb.firebaseio.com"
FIREBASE_AUTH = "AIzaSyD2C_IbPssaeiSZGz69iuG5QBQb3Lk24bI"  # Not needed for Realtime DB but kept for reference

def firebase_get(path):
    """Get data from Firebase"""
    try:
        url = f"{FIREBASE_URL}{path}.json"
        response = urequests.get(url)
        data = response.json()
        response.close()
        return data
    except Exception as e:
        print(f"Firebase GET error: {e}")
        return None

def firebase_put(path, data):
    """Update data in Firebase"""
    try:
        url = f"{FIREBASE_URL}{path}.json"
        headers = {'Content-Type': 'application/json'}
        response = urequests.put(url, data=ujson.dumps(data), headers=headers)
        response.close()
        return True
    except Exception as e:
        print(f"Firebase PUT error: {e}")
        return False

def firebase_patch(path, data):
    """Patch data in Firebase"""
    try:
        url = f"{FIREBASE_URL}{path}.json"
        headers = {'Content-Type': 'application/json'}
        response = urequests.patch(url, data=ujson.dumps(data), headers=headers)
        response.close()
        return True
    except Exception as e:
        print(f"Firebase PATCH error: {e}")
        return False

# ================= GPS =================
gps = UART(1, baudrate=9600, tx=17, rx=16)

def convert_to_degrees(raw, direction):
    if raw == "":
        return None
    raw = float(raw)
    degrees = int(raw / 100)
    minutes = raw - (degrees * 100)
    decimal = degrees + (minutes / 60)
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

print("🛰 Waiting for GPS fix...")

# ================= ULTRASONIC =================
trig = Pin(19, Pin.OUT)   # D19
echo = Pin(18, Pin.IN)    # D18

def get_distance_cm():
    trig.off()
    time.sleep_us(2)
    trig.on()
    time.sleep_us(10)
    trig.off()

    pulse = time_pulse_us(echo, 1, 30000)
    if pulse <= 0:
        return None

    return (pulse * 0.0343) / 2

# ================= MOTOR CONTROL (L298N) =================
# Motor A (Left)
motor_a_in1 = Pin(25, Pin.OUT)
motor_a_in2 = Pin(26, Pin.OUT)
motor_a_enable = PWM(Pin(27), freq=1000)

# Motor B (Right)
motor_b_in3 = Pin(32, Pin.OUT)
motor_b_in4 = Pin(33, Pin.OUT)
motor_b_enable = PWM(Pin(14), freq=1000)

# Set default speed (0-1023)
MOTOR_SPEED = 700

def motor_stop():
    """Stop both motors"""
    motor_a_in1.off()
    motor_a_in2.off()
    motor_b_in3.off()
    motor_b_in4.off()
    motor_a_enable.duty(0)
    motor_b_enable.duty(0)
    print("🛑 Motors STOPPED")

def motor_forward():
    """Move forward"""
    motor_a_in1.on()
    motor_a_in2.off()
    motor_b_in3.on()
    motor_b_in4.off()
    motor_a_enable.duty(MOTOR_SPEED)
    motor_b_enable.duty(MOTOR_SPEED)
    print("⬆️ Motors FORWARD")

def motor_backward():
    """Move backward"""
    motor_a_in1.off()
    motor_a_in2.on()
    motor_b_in3.off()
    motor_b_in4.on()
    motor_a_enable.duty(MOTOR_SPEED)
    motor_b_enable.duty(MOTOR_SPEED)
    print("⬇️ Motors BACKWARD")

def motor_left():
    """Turn left"""
    motor_a_in1.off()
    motor_a_in2.on()
    motor_b_in3.on()
    motor_b_in4.off()
    motor_a_enable.duty(MOTOR_SPEED)
    motor_b_enable.duty(MOTOR_SPEED)
    print("⬅️ Motors LEFT")

def motor_right():
    """Turn right"""
    motor_a_in1.on()
    motor_a_in2.off()
    motor_b_in3.off()
    motor_b_in4.on()
    motor_a_enable.duty(MOTOR_SPEED)
    motor_b_enable.duty(MOTOR_SPEED)
    print("➡️ Motors RIGHT")

# Initialize motors to stopped state
motor_stop()

# ================= MOTOR STATE =================
motor_stopped = False   # False = moving, True = stopped
last_command = "stop"
last_firebase_check = 0
FIREBASE_CHECK_INTERVAL = 500  # milliseconds

# ================= FIREBASE INITIALIZATION =================
# Initialize Firebase nodes
firebase_put("/robotStatus", {
    "obstacle": False,
    "state": "stopped",
    "lastUpdate": time.time()
})

firebase_put("/robotCommands", {
    "move": "stop",
    "timestamp": time.time()
})

firebase_put("/robotGPS", {
    "lat": None,
    "lon": None,
    "lastUpdate": time.time()
})

print("✅ Firebase initialized")

# ================= MAIN LOOP =================
while True:
    current_time = time.ticks_ms()

    # -------- OBSTACLE CHECK --------
    distance = get_distance_cm()
    obstacle_detected = False

    if distance is not None and distance < 20:
        obstacle_detected = True
        if not motor_stopped:
            print("🛑 Motor stopped due to obstacle")
            motor_stop()
            motor_stopped = True
            
            # Update Firebase status
            firebase_patch("/robotStatus", {
                "obstacle": True,
                "state": "stopped",
                "lastUpdate": time.time()
            })
        
        time.sleep(0.5)
        continue
    else:
        if motor_stopped:
            print("🚗 Obstacle cleared, ready for commands")
            motor_stopped = False
            
            # Update Firebase status
            firebase_patch("/robotStatus", {
                "obstacle": False,
                "state": "stopped",
                "lastUpdate": time.time()
            })

    # -------- FIREBASE COMMAND CHECK (every 500ms) --------
    if time.ticks_diff(current_time, last_firebase_check) >= FIREBASE_CHECK_INTERVAL:
        last_firebase_check = current_time
        
        # Get command from Firebase
        command_data = firebase_get("/robotCommands")
        
        if command_data and "move" in command_data:
            new_command = command_data["move"]
            
            # Only execute if command changed and no obstacle
            if new_command != last_command and not motor_stopped:
                last_command = new_command
                
                if new_command == "forward":
                    motor_forward()
                    firebase_patch("/robotStatus", {"state": "moving"})
                    
                elif new_command == "backward":
                    motor_backward()
                    firebase_patch("/robotStatus", {"state": "moving"})
                    
                elif new_command == "left":
                    motor_left()
                    firebase_patch("/robotStatus", {"state": "moving"})
                    
                elif new_command == "right":
                    motor_right()
                    firebase_patch("/robotStatus", {"state": "moving"})
                    
                elif new_command == "stop":
                    motor_stop()
                    firebase_patch("/robotStatus", {"state": "stopped"})

    # -------- GPS READ --------
    if gps.any():
        line = gps.readline()
        if not line:
            continue

        try:
            line = line.decode('utf-8').strip()
        except:
            continue

        if "$GNGGA" in line or "$GPGGA" in line:
            parts = line.split(",")

            if len(parts) < 8:
                continue

            fix_quality = parts[6]

            if fix_quality != "0":
                try:
                    lat = convert_to_degrees(parts[2], parts[3])
                    lon = convert_to_degrees(parts[4], parts[5])
                    sats = parts[7]
                except:
                    continue

                if lat is not None and lon is not None:
                    print("📍 GPS FIXED")
                    print("Latitude:", lat)
                    print("Longitude:", lon)
                    print("Satellites:", sats)
                    print("-------------------------")
                    
                    # Send GPS data to Firebase
                    firebase_patch("/robotGPS", {
                        "lat": lat,
                        "lon": lon,
                        "satellites": sats,
                        "lastUpdate": time.time()
                    })

    time.sleep(0.2)
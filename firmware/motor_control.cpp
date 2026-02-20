"""
KLNavBot — motor_control.py
L298N Motor Driver — MicroPython

YOUR Pin Wiring:
  Motor A (Left)  : IN1=25  IN2=26  ENA=27 (PWM)
  Motor B (Right) : IN3=32  IN4=33  ENB=14 (PWM)
  Default Speed   : 700  (range 0–1023)

Author : KLNavBot Team
Version: 2.0.0
"""

from machine import Pin, PWM


# ═══════════════════════════════════════════════════════════
# PIN DEFINITIONS  (matched to your exact wiring)
# ═══════════════════════════════════════════════════════════

# Motor A — LEFT motor
_IN1 = 25        # direction A
_IN2 = 26        # direction B
_ENA = 27        # PWM speed control

# Motor B — RIGHT motor
_IN3 = 32        # direction A
_IN4 = 33        # direction B
_ENB = 14        # PWM speed control

_PWM_FREQ = 1000  # 1 kHz PWM frequency


# ═══════════════════════════════════════════════════════════
# MOTOR CONTROLLER CLASS
# ═══════════════════════════════════════════════════════════

class MotorController:
    """
    Controls two DC motors through L298N H-bridge.

    Example:
        from motor_control import MotorController
        motors = MotorController()
        motors.forward()
        motors.stop()
    """

    def __init__(self, speed: int = 700):
        """
        Initialise all direction pins and PWM channels.
        speed : default duty cycle (0–1023)
        """
        self.speed = speed

        # ── Direction pins ───────────────────────────────────
        self._in1 = Pin(_IN1, Pin.OUT)   # Left  motor dir A
        self._in2 = Pin(_IN2, Pin.OUT)   # Left  motor dir B
        self._in3 = Pin(_IN3, Pin.OUT)   # Right motor dir A
        self._in4 = Pin(_IN4, Pin.OUT)   # Right motor dir B

        # ── PWM enable pins ──────────────────────────────────
        self._ena = PWM(Pin(_ENA), freq=_PWM_FREQ)  # Left  speed
        self._enb = PWM(Pin(_ENB), freq=_PWM_FREQ)  # Right speed

        # Always safe-stop on startup
        self.stop()
        print("[MOTOR] Ready  —  A: 25/26/27  |  B: 32/33/14")

    # ────────────────────────────────────────────────────────
    # Private helpers
    # ────────────────────────────────────────────────────────

    def _set_dir(self, in1: int, in2: int, in3: int, in4: int):
        """Set all four direction pin logic levels at once."""
        self._in1.value(in1)
        self._in2.value(in2)
        self._in3.value(in3)
        self._in4.value(in4)

    def _apply_speed(self, duty: int = None):
        """
        Apply PWM duty to both enable pins.
        Uses self.speed if duty is not provided.
        Clamps to valid 0–1023 range.
        """
        d = duty if duty is not None else self.speed
        d = max(0, min(1023, d))
        self._ena.duty(d)
        self._enb.duty(d)

    # ────────────────────────────────────────────────────────
    # Public motor commands
    # ────────────────────────────────────────────────────────

    def forward(self, duty: int = None):
        """Drive both motors FORWARD.
        Left : IN1=1 IN2=0 | Right: IN3=1 IN4=0
        """
        self._set_dir(1, 0, 1, 0)
        self._apply_speed(duty)
        print("[MOTOR] >> FORWARD")

    def backward(self, duty: int = None):
        """Drive both motors BACKWARD.
        Left : IN1=0 IN2=1 | Right: IN3=0 IN4=1
        """
        self._set_dir(0, 1, 0, 1)
        self._apply_speed(duty)
        print("[MOTOR] >> BACKWARD")

    def left(self, duty: int = None):
        """Pivot LEFT in place.
        Left motor BACKWARD, Right motor FORWARD.
        """
        self._set_dir(0, 1, 1, 0)
        self._apply_speed(duty)
        print("[MOTOR] >> LEFT")

    def right(self, duty: int = None):
        """Pivot RIGHT in place.
        Left motor FORWARD, Right motor BACKWARD.
        """
        self._set_dir(1, 0, 0, 1)
        self._apply_speed(duty)
        print("[MOTOR] >> RIGHT")

    def stop(self):
        """Immediately cut power to both motors."""
        self._set_dir(0, 0, 0, 0)
        self._ena.duty(0)
        self._enb.duty(0)
        print("[MOTOR] >> STOPPED")

    def set_speed(self, duty: int):
        """Update the default running speed (0–1023)."""
        self.speed = max(0, min(1023, duty))
        print(f"[MOTOR] Speed set → {self.speed}")

    def deinit(self):
        """Release PWM hardware resources before deep sleep."""
        self.stop()
        self._ena.deinit()
        self._enb.deinit()
        print("[MOTOR] PWM released")

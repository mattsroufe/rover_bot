import time
import board
import pwmio
from adafruit_motor import servo, motor
import adafruit_hcsr04
import simpleio
import neopixel

# === Configuration ===

# Trim values for servo center calibration (-90 to +90)
PAN_TRIM = 30     # Adjust this to center pan
TILT_TRIM = 25   # Adjust this to tilt camera up/down

# Obstacle distance threshold in cm
OBSTACLE_THRESHOLD_CM = 50

# Servo center angle (logical midpoint)
CENTER_ANGLE = 90

# Relative scan angles (offsets from center)
LEFT_SCAN_OFFSET = -50
RIGHT_SCAN_OFFSET = +50

# Motor directions
MOTOR1_REVERSED = True
MOTOR2_REVERSED = True

# === Setup hardware ===

# Neopixel RGB LEDs
pixels = neopixel.NeoPixel(board.GP18, 2)
pixels.brightness = 0.5

def set_color(color):
    pixels.fill(color)

# Sounds
PIEZO_PIN = board.GP22

def play_tone(freq, duration=0.15):
    simpleio.tone(PIEZO_PIN, freq, duration)

def play_startup_melody():
    notes = [659, 659, 0, 659, 0, 523, 659, 0, 784]
    for freq in notes:
        if freq > 0:
            play_tone(freq, 0.15)
        else:
            time.sleep(0.15)

def beep_obstacle():
    play_tone(300, 0.1)
    play_tone(200, 0.1)

def beep_turn_left():
    play_tone(440, 0.2)

def beep_turn_right():
    play_tone(660, 0.2)

def beep_resume():
    play_tone(880, 0.1)
    play_tone(1000, 0.1)

# Servos
pwm_pan = pwmio.PWMOut(board.GP13, duty_cycle=0, frequency=50)
pwm_tilt = pwmio.PWMOut(board.GP12, duty_cycle=0, frequency=50)
pan_servo = servo.Servo(pwm_pan)
tilt_servo = servo.Servo(pwm_tilt)

def set_pan(relative_angle):
    """Set pan angle with center + trim + relative offset"""
    angle = CENTER_ANGLE + PAN_TRIM + relative_angle
    pan_servo.angle = max(0, min(180, angle))

def set_tilt(relative_angle):
    """Set tilt angle with center + trim + relative offset"""
    angle = CENTER_ANGLE + TILT_TRIM + relative_angle
    tilt_servo.angle = max(0, min(180, angle))

# Ultrasonic distance sensor
sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.GP0, echo_pin=board.GP1)

# Motors
M1A = pwmio.PWMOut(board.GP8, frequency=10000)
M1B = pwmio.PWMOut(board.GP9, frequency=10000)
M2A = pwmio.PWMOut(board.GP10, frequency=10000)
M2B = pwmio.PWMOut(board.GP11, frequency=10000)
motor1 = motor.DCMotor(M1A, M1B)
motor2 = motor.DCMotor(M2A, M2B)

def stop_motors():
    motor1.throttle = 0
    motor2.throttle = 0

def move_forward(speed=0.8):
    motor1.throttle = -speed if MOTOR1_REVERSED else speed
    motor2.throttle = -speed if MOTOR2_REVERSED else speed

def turn_left(duration=0.6, speed=0.7):
    beep_turn_left()
    motor1.throttle = speed if not MOTOR1_REVERSED else -speed
    motor2.throttle = -speed if not MOTOR2_REVERSED else speed
    time.sleep(duration)
    stop_motors()

def turn_right(duration=0.6, speed=0.7):
    beep_turn_right()
    motor1.throttle = -speed if not MOTOR1_REVERSED else speed
    motor2.throttle = speed if not MOTOR2_REVERSED else -speed
    time.sleep(duration)
    stop_motors()

# === Scanning ===

def scan_distance(offset_angle):
    """Scan at a relative angle offset from pan center"""
    set_pan(offset_angle)
    set_tilt(0)
    time.sleep(0.3)
    try:
        distance_cm = sonar.distance
    except RuntimeError:
        distance_cm = -1
    print(f"Scan {offset_angle:+}° → {distance_cm} cm")
    return distance_cm

# === Main program ===

play_startup_melody()
print("Robot starting...")
set_pan(0)
set_tilt(0)

while True:
    set_color((0, 255, 0))  # 💚 Moving
    move_forward()
    time.sleep(0.1)

    try:
        front_distance = sonar.distance
    except RuntimeError:
        front_distance = -1

    print(f"Front: {front_distance} cm")

    if 0 < front_distance < OBSTACLE_THRESHOLD_CM:
        print("🚫 Obstacle!")
        set_color((255, 0, 0))  # 🔴
        beep_obstacle()
        stop_motors()
        time.sleep(0.3)

        set_color((255, 255, 0))  # 🟡 Scanning
        left_distance = scan_distance(LEFT_SCAN_OFFSET)
        right_distance = scan_distance(RIGHT_SCAN_OFFSET)

        set_pan(0)  # Reset to center

        if left_distance == -1 and right_distance == -1:
            print("⚪ No valid readings")
            set_color((255, 255, 255))
            stop_motors()
            break

        set_color((0, 0, 255))  # 🔵 Turning
        if left_distance > right_distance:
            print(f"↩️ Left: {left_distance} cm")
            turn_left()
        else:
            print(f"↪️ Right: {right_distance} cm")
            turn_right()

        beep_resume()
        print("✅ Resuming...")
        time.sleep(0.2)


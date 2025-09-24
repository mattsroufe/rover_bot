import time
import board
import pwmio
from adafruit_motor import motor
import adafruit_hcsr04
import simpleio
import neopixel

# === Configuration ===

OBSTACLE_THRESHOLD_CM = 30         # Distance to stop and turn
WALL_FOLLOW_THRESHOLD_CM = 60      # Start steering correction if closer than this
NORMAL_SPEED = 0.8
CORRECTION_SPEED = 0.5
TURN_DURATION = 0.6
MOTOR1_REVERSED = True
MOTOR2_REVERSED = True

# === Setup Hardware ===

# Neopixels
pixels = neopixel.NeoPixel(board.GP18, 2)
pixels.brightness = 0.5

def set_color(color):
    pixels.fill(color)

# Buzzer (Piezo)
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

# Ultrasonic sensors
sonar_front = adafruit_hcsr04.HCSR04(trigger_pin=board.GP0, echo_pin=board.GP1)
sonar_left = adafruit_hcsr04.HCSR04(trigger_pin=board.GP2, echo_pin=board.GP3)
sonar_right = adafruit_hcsr04.HCSR04(trigger_pin=board.GP7, echo_pin=board.GP28)

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

def set_motors(left_speed, right_speed):
    motor1.throttle = -left_speed if MOTOR1_REVERSED else left_speed
    motor2.throttle = -right_speed if MOTOR2_REVERSED else right_speed

def turn_left(duration=TURN_DURATION, speed=0.7):
    beep_turn_left()
    set_motors(speed, -speed)
    time.sleep(duration)
    stop_motors()

def turn_right(duration=TURN_DURATION, speed=0.7):
    beep_turn_right()
    set_motors(-speed, speed)
    time.sleep(duration)
    stop_motors()

# === Main Program ===

play_startup_melody()
print("Robot starting...")

while True:
    time.sleep(0.05)
    current_time = time.monotonic()

    # Read sonar sensors
    try:
        front = sonar_front.distance
    except RuntimeError:
        front = -1

    try:
        left = sonar_left.distance
    except RuntimeError:
        left = -1

    try:
        right = sonar_right.distance
    except RuntimeError:
        right = -1

    # Log readings
    print(f"[{current_time:.2f}s] Front: {front} cm | Left: {left} cm | Right: {right} cm")

    # === Obstacle directly in front ===
    if 0 < front < OBSTACLE_THRESHOLD_CM:
        print(f"[{current_time:.2f}s] 🚫 Obstacle ahead! Stopping.")
        set_color((255, 0, 0))  # Red
        stop_motors()
        beep_obstacle()
        time.sleep(0.3)

        # Determine better turn direction
        if (left > right) or right == -1:
            print(f"[{current_time:.2f}s] ↩️ Turning left (more space: {left} cm)")
            set_color((0, 0, 255))  # Blue
            turn_left()
        else:
            print(f"[{current_time:.2f}s] ↪️ Turning right (more space: {right} cm)")
            set_color((0, 0, 255))  # Blue
            turn_right()

        beep_resume()
        print(f"[{current_time:.2f}s] ✅ Resuming forward motion...")
        continue  # Skip forward logic, restart loop

    # === Wall-following via gentle steering ===
    set_color((0, 255, 0))  # Green

    left_speed = NORMAL_SPEED
    right_speed = NORMAL_SPEED

    if 0 < left < WALL_FOLLOW_THRESHOLD_CM:
        print(f"[{current_time:.2f}s] ⚠️ Wall on left ({left} cm). Steering right.")
        left_speed = CORRECTION_SPEED
        right_speed = NORMAL_SPEED

    elif 0 < right < WALL_FOLLOW_THRESHOLD_CM:
        print(f"[{current_time:.2f}s] ⚠️ Wall on right ({right} cm). Steering left.")
        left_speed = NORMAL_SPEED
        right_speed = CORRECTION_SPEED

    set_motors(left_speed, right_speed)


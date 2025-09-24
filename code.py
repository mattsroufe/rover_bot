import time
import board
import pwmio
import simpleio
import neopixel
import adafruit_hcsr04
from adafruit_motor import motor

# === Configuration ===
OBSTACLE_THRESHOLD_CM = 30     # Distance to stop and turn
WALL_FOLLOW_THRESHOLD_CM = 60  # Distance to start steering correction
NORMAL_SPEED = 0.8
CORRECTION_SPEED = 0.5
TURN_DURATION = 0.6
MOTOR1_REVERSED = True
MOTOR2_REVERSED = True

# === Setup Hardware ===
# Neopixels
pixels = neopixel.NeoPixel(board.GP18, 2, brightness=0.5)

def set_color(color):
    pixels.fill(color)

# Buzzer (Piezo)
PIEZO_PIN = board.GP22

def play_tone(freq, duration=0.12, color=None):
    """Play a tone with optional color flash."""
    if color:
        set_color(color)
    simpleio.tone(PIEZO_PIN, freq, duration)

def play_melody(notes, tempo=0.12, color=None, reset_color=(0, 255, 0)):
    """Play a sequence of notes, flash LEDs. 0 means rest."""
    for freq in notes:
        if freq > 0:
            play_tone(freq, tempo, color=color)
        else:
            set_color(reset_color)
            time.sleep(tempo)
    set_color(reset_color)  # return to normal state

# === Sound effects ===
def play_startup_melody():
    """Cheerful ascending arpeggio (power on)."""
    notes = [523, 659, 784, 1047]  # C-E-G-C'
    play_melody(notes, tempo=0.15, color=(0, 0, 255))  # Blue

def play_shutdown_melody():
    """Descending tone (power down)."""
    notes = [1047, 784, 659, 523]
    play_melody(notes, tempo=0.15, color=(128, 0, 128))  # Purple

def beep_obstacle():
    """Urgent alarm beep (obstacle ahead)."""
    play_melody([600, 500, 400], tempo=0.08, color=(255, 0, 0))  # Red

def beep_turn_left():
    """Two quick descending chirps (left turn)."""
    play_melody([700, 500, 0, 700, 500], tempo=0.07, color=(0, 0, 255))  # Blue

def beep_turn_right():
    """Two quick ascending chirps (right turn)."""
    play_melody([500, 700, 0, 500, 700], tempo=0.07, color=(0, 0, 255))  # Blue

def beep_resume():
    """Happy rising chirp (resume forward)."""
    play_melody([500, 700, 900], tempo=0.08, color=(0, 255, 0))  # Green

# === Sensors ===
sonar_front = adafruit_hcsr04.HCSR04(trigger_pin=board.GP0, echo_pin=board.GP1)
sonar_left  = adafruit_hcsr04.HCSR04(trigger_pin=board.GP2, echo_pin=board.GP3)
sonar_right = adafruit_hcsr04.HCSR04(trigger_pin=board.GP7, echo_pin=board.GP28)

def read_distance(sensor):
    """Safely read sonar distance, return -1 if unavailable."""
    try:
        return sensor.distance
    except RuntimeError:
        return -1

# === Motors ===
M1A, M1B = pwmio.PWMOut(board.GP8, 10000), pwmio.PWMOut(board.GP9, 10000)
M2A, M2B = pwmio.PWMOut(board.GP10, 10000), pwmio.PWMOut(board.GP11, 10000)
motor1, motor2 = motor.DCMotor(M1A, M1B), motor.DCMotor(M2A, M2B)

def stop_motors():
    motor1.throttle = 0
    motor2.throttle = 0

def set_motors(left_speed, right_speed):
    motor1.throttle = -left_speed if MOTOR1_REVERSED else left_speed
    motor2.throttle = -right_speed if MOTOR2_REVERSED else right_speed

def turn_left(duration=TURN_DURATION, speed=0.7):
    """Turn left with LED blinking and sound."""
    beep_turn_left()
    blink_end = time.monotonic() + duration
    while time.monotonic() < blink_end:
        set_color((0, 0, 255))  # Blue on
        set_motors(speed, -speed)
        time.sleep(0.2)
        set_color((0, 0, 0))  # Off (blink)
        time.sleep(0.2)
    stop_motors()

def turn_right(duration=TURN_DURATION, speed=0.7):
    """Turn right with LED blinking and sound."""
    beep_turn_right()
    blink_end = time.monotonic() + duration
    while time.monotonic() < blink_end:
        set_color((0, 0, 255))  # Blue on
        set_motors(-speed, speed)
        time.sleep(0.2)
        set_color((0, 0, 0))  # Off (blink)
        time.sleep(0.2)
    stop_motors()

# === Behaviors ===
def read_sensors():
    """Read all sonar sensors and return dict of distances (cm)."""
    return {
        "front": read_distance(sonar_front),
        "left":  read_distance(sonar_left),
        "right": read_distance(sonar_right),
    }

def log_sensors(current_time, sensors):
    print(
        f"[{current_time:.2f}s] "
        f"Front: {sensors['front']} cm | "
        f"Left: {sensors['left']} cm | "
        f"Right: {sensors['right']} cm"
    )

def handle_obstacle(current_time, sensors):
    """Handle obstacle avoidance. Returns True if obstacle was handled."""
    front, left, right = sensors["front"], sensors["left"], sensors["right"]

    if 0 < front < OBSTACLE_THRESHOLD_CM:
        print(f"[{current_time:.2f}s] Obstacle ahead! Stopping.")
        set_color((255, 0, 0))  # Red
        stop_motors()
        beep_obstacle()
        time.sleep(0.3)

        # Pick direction
        if (left > right) or right == -1:
            print(f"[{current_time:.2f}s] Turning left (more space: {left} cm)")
            turn_left()
        else:
            print(f"[{current_time:.2f}s] Turning right (more space: {right} cm)")
            turn_right()

        beep_resume()
        print(f"[{current_time:.2f}s] Resuming forward motion...")
        return True

    return False

def handle_wall_following(current_time, sensors):
    """Adjust motor speeds for wall following."""
    left, right = sensors["left"], sensors["right"]
    set_color((0, 255, 0))  # Green (normal driving)

    left_speed, right_speed = NORMAL_SPEED, NORMAL_SPEED

    if 0 < left < WALL_FOLLOW_THRESHOLD_CM:
        print(f"[{current_time:.2f}s] Wall on left ({left} cm). Steering right.")
        left_speed = CORRECTION_SPEED
    elif 0 < right < WALL_FOLLOW_THRESHOLD_CM:
        print(f"[{current_time:.2f}s] Wall on right ({right} cm). Steering left.")
        right_speed = CORRECTION_SPEED

    set_motors(left_speed, right_speed)

# === Main Program ===
def main():
    play_startup_melody()
    print("Robot starting...")

    while True:
        time.sleep(0.05)
        current_time = time.monotonic()

        sensors = read_sensors()
        log_sensors(current_time, sensors)

        # Obstacle takes priority
        if handle_obstacle(current_time, sensors):
            continue

        # Otherwise wall-follow
        handle_wall_following(current_time, sensors)

# Run program
main()

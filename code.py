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
NORMAL_SPEED = 1.0
CORRECTION_SPEED = 0.8
TURN_DURATION = 0.6
MOTOR1_REVERSED = True
MOTOR2_REVERSED = True

# === Setup Hardware ===
# Neopixels
pixels = neopixel.NeoPixel(board.GP18, 2, brightness=0.5)

def set_color(color):
    pixels.fill(color)

def wheel(pos):
    """Generate rainbow colors across 0–255 positions."""
    if pos < 85:
        return (255 - pos * 3, pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (0, 255 - pos * 3, pos * 3)
    else:
        pos -= 170
        return (pos * 3, 0, 255 - pos * 3)

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

# === Driving music melody (Nyan Cat loop) ===
# Each entry is (frequency Hz, duration s).
nyan_melody = [
    (659, 0.15), (659, 0.15), (0, 0.15), (659, 0.15),
    (523, 0.15), (659, 0.15), (784, 0.3),
    (392, 0.3),
    (523, 0.15), (392, 0.15), (330, 0.15), (440, 0.15),
    (494, 0.15), (440, 0.15), (330, 0.15), (392, 0.15),
    (659, 0.15), (784, 0.3), (880, 0.3),
    (698, 0.15), (784, 0.15), (659, 0.15),
]

def play_driving_melody_step(step_index, rainbow_index):
    """Play one step of the Nyan Cat melody + rainbow LEDs."""
    freq, duration = nyan_melody[step_index]
    set_color(wheel(rainbow_index % 255))  # rainbow cycle
    if freq > 0:
        simpleio.tone(PIEZO_PIN, freq, duration)
    else:
        time.sleep(duration)

# === Sound effects ===
def play_startup_melody():
    notes = [523, 659, 784, 1047]
    play_melody(notes, tempo=0.15, color=(0, 0, 255))  # Blue

def play_shutdown_melody():
    notes = [1047, 784, 659, 523]
    play_melody(notes, tempo=0.15, color=(128, 0, 128))  # Purple

def beep_obstacle():
    play_melody([600, 500, 400], tempo=0.08, color=(255, 0, 0))  # Red

def beep_turn_left():
    play_melody([700, 500, 0, 700, 500], tempo=0.07, color=(0, 0, 255))  # Blue

def beep_turn_right():
    play_melody([500, 700, 0, 500, 700], tempo=0.07, color=(0, 0, 255))  # Blue

def beep_resume():
    play_melody([500, 700, 900], tempo=0.08, color=(0, 255, 0))  # Green

# === Sensors ===
sonar_front = adafruit_hcsr04.HCSR04(trigger_pin=board.GP0, echo_pin=board.GP1)
sonar_left  = adafruit_hcsr04.HCSR04(trigger_pin=board.GP2, echo_pin=board.GP3)
sonar_right = adafruit_hcsr04.HCSR04(trigger_pin=board.GP7, echo_pin=board.GP28)

def read_distance(sensor):
    try:
        return sensor.distance
    except RuntimeError:
        return -1

# === Motors ===
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
    blink_end = time.monotonic() + duration
    while time.monotonic() < blink_end:
        set_color((0, 0, 255))
        set_motors(speed, -speed)
        time.sleep(0.2)
        set_color((0, 0, 0))
        time.sleep(0.2)
    stop_motors()

def turn_right(duration=TURN_DURATION, speed=0.7):
    beep_turn_right()
    blink_end = time.monotonic() + duration
    while time.monotonic() < blink_end:
        set_color((0, 0, 255))
        set_motors(-speed, speed)
        time.sleep(0.2)
        set_color((0, 0, 0))
        time.sleep(0.2)
    stop_motors()

# === Behaviors ===
def read_sensors():
    return {
        "front": read_distance(sonar_front),
        "left":  read_distance(sonar_left),
        "right": read_distance(sonar_right),
    }

def log_sensors(current_time, sensors):
    print(f"[{current_time:.2f}s] Front: {sensors['front']} cm | Left: {sensors['left']} cm | Right: {sensors['right']} cm")

def handle_obstacle(current_time, sensors):
    front, left, right = sensors["front"], sensors["left"], sensors["right"]

    if 0 < front < OBSTACLE_THRESHOLD_CM:
        print(f"[{current_time:.2f}s] Obstacle ahead! Stopping.")
        set_color((255, 0, 0))
        stop_motors()
        beep_obstacle()
        time.sleep(0.3)

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
    left, right = sensors["left"], sensors["right"]

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
    music_step = 0
    last_music_time = time.monotonic()
    rainbow_index = 0

    while True:
        time.sleep(0.01)
        current_time = time.monotonic()
        sensors = read_sensors()
        log_sensors(current_time, sensors)

        if handle_obstacle(current_time, sensors):
            continue

        # Play Nyan Cat step & rainbow sync
        freq, duration = nyan_melody[music_step]
        if current_time - last_music_time >= duration:
            play_driving_melody_step(music_step, rainbow_index)
            last_music_time = current_time
            music_step = (music_step + 1) % len(nyan_melody)
            rainbow_index = (rainbow_index + 8) % 255  # advance rainbow

        handle_wall_following(current_time, sensors)

# Run program
main()

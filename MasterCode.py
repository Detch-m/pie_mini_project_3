"""Master controlling Node.

Reads people-tracking data from the Seeed MR60BHA2 mmWave sensor on the XiaoC6
ESP32 (mmwave_xy.ino). Two independent behaviours, both sent to the Uno sketch
(motor_servo.ino):

Head (DC motor), open-loop "glance" gesture, per detection:
  1. angle = atan2(x, y)  -> 0 deg straight ahead, +/- to the sides.
     dist  = hypot(x, y)  -> metres from the sensor.
  2. Pick the nearest person within MAX_RANGE_M. Ignore anyone wider than
     +/-MAX_ANGLE_DEG (ghost) or closer to centre than MIN_ANGLE_DEG (too close).
  3. Wait HOLD_S seconds, then send HOME_DEG -> the head returns to where it
     started.

Arm (servo), simple waving / not waving:
  - Someone within WAVE_RANGE_M -> send "s"
    (one wave), and remember that we waved.
  - Nobody within WAVE_RANGE_M -> forget it, so the next approach gets a wave.

Idle (audio), play a song when left alone:
  - Nobody within MAX_RANGE_M for IDLE_S seconds -> play a track through
    pygame. The two tracks alternate.
  - If still alone, wait IDLE_S after the track ends and play the other one.
  - Anyone stepping within MAX_RANGE_M stops the track and resumes glancing.

The Uno does one thing at a time: if a glance and a wave are triggered
together, it turns the head first, then waves, then returns home.
"""

import math
import os
import time

import pygame
import serial

MMWAVE_PORT = "COM9"
MOTOR_PORT = "COM5"
BAUD = 115200

MAX_RANGE_M = 1.5
MAX_ANGLE_DEG = 60.0
MIN_ANGLE_DEG = 15.0
HOME_DEG = 0.0
HOLD_S = 2.0

WAVE_RANGE_M = 1.0

SETTLE_S = 2.0

IDLE_S = 15.0
_HERE = os.path.dirname(os.path.abspath(__file__))
SONGS = [
    os.path.join(_HERE, "MegaWalls_Technoblade.mp3"),
    os.path.join(_HERE, "Blitz_Technoblade.mp3"),
]


def parse_targets(line):
    """'x,y,x,y,...' returns [(x, y), ...] in meters. [] if nothing."""
    if not line:
        return []
    try:
        values = [float(v) for v in line.split(",")]
    except ValueError:
        return []
    if len(values) % 2 != 0:  # assert that they come in pairs
        return []
    return list(zip(values[0::2], values[1::2]))


def choose_angle(targets):
    """Angle to glance at, or None if nobody worth turning for is in range."""
    in_range = [
        (math.hypot(x, y), x, y) for x, y in targets if math.hypot(x, y) <= MAX_RANGE_M
    ]
    if not in_range:
        return None
    _, x, y = min(in_range)  # the nearest person
    angle = math.degrees(math.atan2(x, y))
    if abs(angle) > MAX_ANGLE_DEG:
        return None
    if abs(angle) < MIN_ANGLE_DEG:
        return None
    return angle


def anyone_close(targets):
    """True if anyone is closer than WAVE_RANGE_M."""
    for x, y in targets:
        if math.hypot(x, y) < WAVE_RANGE_M:
            return True
    return False


def anyone_in_range(targets):
    """True if anyone is within MAX_RANGE_M -> 'someone is here'.

    Independent of the glance angle filters: a person off to the side still
    counts as present, so their being there resets the idle timer.
    """
    for x, y in targets:
        if math.hypot(x, y) <= MAX_RANGE_M:
            return True
    return False


def send_angle(motor, deg):
    """Send one open-loop move command: a number + newline."""
    motor.write(f"{deg:.1f}\n".encode())


def send_wave(motor):
    """Tell the Uno to wave once: the letter s + newline."""
    motor.write("s\n".encode())


def main():
    """Watch the sensor; glance at the nearest person and wave at anyone close."""
    sensor = serial.Serial(MMWAVE_PORT, BAUD, timeout=1)
    motor = serial.Serial(MOTOR_PORT, BAUD, timeout=1)
    time.sleep(SETTLE_S)
    sensor.reset_input_buffer()
    motor.write(b"z\n")
    time.sleep(0.2)
    pygame.mixer.init()
    waved = False
    last_seen = time.monotonic()  # Timer
    song_index = 0  # alternates
    try:
        while True:
            line = sensor.readline().decode(errors="ignore").strip()
            targets = parse_targets(line)
            angle = choose_angle(targets)
            present = anyone_in_range(targets)
            playing = bool(pygame.mixer.get_init()) and pygame.mixer.music.get_busy()
            now = time.monotonic()

            if present and playing:
                pygame.mixer.music.stop()
                playing = False

            if present or playing:
                last_seen = now

            # Tilt Head
            if angle is not None:
                send_angle(motor, angle * -1)  # glance at them
                print(f"glance -> {angle:+.1f} deg")

            # Wave
            if anyone_close(targets):
                if not waved:
                    send_wave(motor)
                    print("wave")
                    waved = True
            else:
                waved = False

            # Play a song
            if not present and not playing and now - last_seen >= IDLE_S:
                song = SONGS[song_index % len(SONGS)]
                pygame.mixer.music.load(song)
                pygame.mixer.music.play()
                print("playing song")
                song_index += 1
                last_seen = now  # keep the clock warm until this one ends

            # Hold and Return Head
            if angle is not None:
                time.sleep(HOLD_S)
                send_angle(motor, HOME_DEG)
                print(f"return -> {HOME_DEG:+.1f} deg")
                sensor.reset_input_buffer()

    except KeyboardInterrupt:
        print("\nStopping")

    finally:
        if pygame.mixer.get_init():
            pygame.mixer.music.stop()
            pygame.mixer.quit()
        try:
            send_angle(motor, HOME_DEG)
            time.sleep(0.2)
        except serial.SerialException:
            print("Check if Ports have an error")
        sensor.close()
        motor.close()


if __name__ == "__main__":
    main()

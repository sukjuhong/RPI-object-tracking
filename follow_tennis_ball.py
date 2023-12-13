# You need to turn on pigpio daemon.
# To start the pigpio daemon, execute <sudo pigpiod> in terminal.

import time
import RPi.GPIO as GPIO
import pigpio
import cv2
from picamera2 import Picamera2

CHANNEL_RAZER = 25
CHANNEL_PAN_MOTOR = 17
CHANNEL_TILT_MOTOR = 18

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

GREEN_LOWER = (29, 86, 6)
GREEN_UPPER = (64, 255, 255)

GPIO.setmode(GPIO.BCM)

piCam = Picamera2()
piCam.preview_configuration.main.size = (CAMERA_WIDTH, CAMERA_HEIGHT)
piCam.preview_configuration.main.format = "RGB888"
piCam.configure("preview")
piCam.start()

pi = pigpio.pi()


def current_milli_time():
    return round(time.time() * 1000)


class Motor:
    channel = -1
    cur_angle = 90

    def __init__(self, channel: int):
        self.channel = channel
        self.change_duty_cycle(self.cur_angle)

    def change_duty_cycle(self, angle: int):
        if angle < 0 or angle > 180:
            print("out of range")
            return
        self.cur_angle = angle
        pulsewidth = 600 + 10 * angle
        pi.set_servo_pulsewidth(self.channel, pulsewidth)


class Razer:
    channel = 0

    def __init__(self, channel: int):
        self.channel = channel
        GPIO.setup(self.channel, GPIO.OUT)

    def turn_on(self):
        print("Razer is turned on.")
        GPIO.output(self.channel, GPIO.HIGH)

    def turn_off(self):
        print("Razer is turned off.")
        GPIO.output(self.channel, GPIO.LOW)


if __name__ == "__main__":
    pan_motor = Motor(CHANNEL_PAN_MOTOR)
    tilt_motor = Motor(CHANNEL_TILT_MOTOR)

    razer = Razer(CHANNEL_RAZER)
    razer.turn_on()

    i_term = (0, 0)
    last_error = (0, 0)
    previous_time = current_milli_time()

    kP = 0.028
    kD = 3

    while True:
        frame = piCam.capture_array()
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0]

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), r) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            error = (center[0] - CAMERA_WIDTH // 2, center[1] - CAMERA_HEIGHT // 2)
            current_time = current_milli_time()
            dT = current_time - previous_time

            p_term = error
            d_term = ((error[0] - last_error[0]) / dT, (error[1] - last_error[1]) / dT)

            nxt_pan_angle = pan_motor.cur_angle - (p_term[0] * kP + d_term[0] * kD)
            nxt_tilt_angle = tilt_motor.cur_angle + (p_term[1] * kP + d_term[1] * kD)

            pan_motor.change_duty_cycle(nxt_pan_angle)
            tilt_motor.change_duty_cycle(nxt_tilt_angle)

            if r > 10:
                cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 255), 2)

            last_error = error
            previous_time = current_time

        cv2.circle(frame, (CAMERA_WIDTH // 2, CAMERA_HEIGHT // 2), 5, (0, 0, 255), 2)
        cv2.imshow("circle", frame)

        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
GPIO.cleanup()

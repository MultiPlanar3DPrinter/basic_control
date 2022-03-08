#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import math


ROTATER_LENGTH = 10
DEG_PER_STEP = 1.8
TURN_GEAR_RATIO = 2
TILT_GEAR_RATIO = 2*3




def cleanup(steppers):
    for s in steppers:
        s.disable()
    GPIO.cleanup()


class Stepper:
    def __init__(self, pins, half_steps=False):
        self.pins = pins
        self._setup()
        self.disable()
        if half_steps:
            sequence = [[] for _ in range(8)]
            sequence[0] = [1,0,0,0]
            sequence[1] = [1,1,0,0]
            sequence[2] = [0,1,0,0]
            sequence[3] = [0,1,1,0]
            sequence[4] = [0,0,1,0]
            sequence[5] = [0,0,1,1]
            sequence[6] = [0,0,0,1]
            sequence[7] = [1,0,0,1]
            self.sequence = sequence
            self.scale = 2
        else:
            sequence = [[] for _ in range(4)]
            sequence[0] = [1,0,0,0]
            sequence[1] = [0,1,0,0]
            sequence[2] = [0,0,1,0]
            sequence[3] = [0,0,0,1]
            self.sequence = sequence
            self.scale = 1

    def _setup(self):
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)

    def disable(self):
        self.cur_idx = None
        for pin in self.pins:
            GPIO.output(pin, GPIO.LOW)

    def hold(self):
        self.cur_idx = None
        for pin in self.pins:
            GPIO.output(pin, GPIO.HIGH)

    def _output_at_seq_idx(self, seq_idx):
        for pin_idx in range(4):
            pin = self.pins[pin_idx]
            if self.sequence[seq_idx][pin_idx] != 0:
                GPIO.output(pin, GPIO.HIGH)
            else:
                GPIO.output(pin, GPIO.LOW)
    
    def _update_seq_idx(self, sign):
        seq_len = len(self.sequence)
        self.cur_idx += sign
        if (self.cur_idx == seq_len):
            self.cur_idx = 0
        elif (self.cur_idx < 0):
            self.cur_idx = seq_len - 1

    def rotate(self, steps):
        if steps < 0:
            sign = -1
        else:
            sign = 1
        steps = sign * steps * self.scale

        if self.cur_idx is None:
            self.cur_idx = 0

        for _ in range(steps):
            self._update_seq_idx(sign)
            self._output_at_seq_idx(self.cur_idx)

            # change this carefully, might encounter mechanical (motor) limit
            time.sleep(0.008)


def angle(turn_stepper, tilt_stepper, i, j, k):
    alpha = math.atan2(i, j)
    beta = math.asin(k/ROTATER_LENGTH)
    turn_stepper.rotate(alpha*TURN_GEAR_RATIO/DEG_PER_STEP)
    tilt_stepper.rotate(beta*TILT_GEAR_RATIO/DEG_PER_STEP)


def main():
    out1 = 22
    out2 = 17
    out3 = 27
    out4 = 23
    STEP_PINS = [out1, out4, out2, out3] # (black red green blue) or (1 4 2 3)
    # STEP_PINS = [27, 22, 23, 17]
    # STEP_PINS = [23, 17, 27, 22]
    # STEP_PINS = [17, 27, 22, 23]

    GPIO.setmode(GPIO.BCM)
    turn = Stepper(STEP_PINS, half_steps=True)
    tilt = Stepper(None, half_steps=True)
    print("will run in 0.5s")
    time.sleep(0.5)
    try:
        angle(turn, tilt, 0, 0, 0)
    except KeyboardInterrupt:
        cleanup([turn, tilt])
        exit( 1 )

    cleanup([turn, tilt])
    exit( 0 )


if __name__ == "__main__":
    main()

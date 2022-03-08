#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

out1 = 22
out2 = 17
out3 = 27
out4 = 23
STEP_PINS = [out1, out4, out2, out3] # (black red green blue) or (1 4 2 3)
# STEP_PINS = [27, 22, 23, 17]
# STEP_PINS = [23, 17, 27, 22]
# STEP_PINS = [17, 27, 22, 23]

def setup():
    GPIO.setmode( GPIO.BCM )
    GPIO.setup( out1, GPIO.OUT )
    GPIO.setup( out2, GPIO.OUT )
    GPIO.setup( out3, GPIO.OUT )
    GPIO.setup( out4, GPIO.OUT )


def initialize():
    GPIO.output( out1, GPIO.LOW )
    GPIO.output( out2, GPIO.LOW )
    GPIO.output( out3, GPIO.LOW )
    GPIO.output( out4, GPIO.LOW )


def cleanup():
    initialize()
    GPIO.cleanup()


class Stepper:
    def __init__(self, half_steps=False):
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

    def _output_at_seq_idx(self, seq_idx):
        for pin_idx in range(4):
            pin = STEP_PINS[pin_idx]
            if self.sequence[seq_idx][pin_idx] != 0:
                GPIO.output(pin, GPIO.HIGH)
            else:
                GPIO.output(pin, GPIO.LOW)

    def rotate(self, steps):
        if steps < 0:
            sign = -1
        else:
            sign = 1
        steps = sign * steps * self.scale

        seq_len = len(self.sequence)
        seq_idx = 0
        for _ in range(steps):
            self._output_at_seq_idx(seq_idx)

            seq_idx += sign
            if (seq_idx == seq_len):
                seq_idx = 0
            elif (seq_idx < 0):
                seq_idx = seq_len - 1

            # change this carefully, might encounter mechanical (motor) limit
            time.sleep(0.008)

def main():
    setup()
    initialize()
    print("will run in 0.5s")
    time.sleep(0.5)
    try:
        Stepper(half_steps=True).rotate(500)
    except KeyboardInterrupt:
        cleanup()
        exit( 1 )
 
    cleanup()
    exit( 0 )


def angle():
    pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

out1 = 17
out2 = 18
out3 = 27
out4 = 22


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


def rotate(steps):
    # careful lowering this, at some point you run into the mechanical limitation of how quick your motor can move
    step_sleep = 0.002
    for i in range(steps):
        if i%4==0:
            GPIO.output( out4, GPIO.HIGH )
            GPIO.output( out3, GPIO.LOW )
            GPIO.output( out2, GPIO.LOW )
            GPIO.output( out1, GPIO.LOW )
        elif i%4==1:
            GPIO.output( out4, GPIO.LOW )
            GPIO.output( out3, GPIO.LOW )
            GPIO.output( out2, GPIO.HIGH )
            GPIO.output( out1, GPIO.LOW )
        elif i%4==2:
            GPIO.output( out4, GPIO.LOW )
            GPIO.output( out3, GPIO.HIGH )
            GPIO.output( out2, GPIO.LOW )
            GPIO.output( out1, GPIO.LOW )
        elif i%4==3:
            GPIO.output( out4, GPIO.LOW )
            GPIO.output( out3, GPIO.LOW )
            GPIO.output( out2, GPIO.LOW )
            GPIO.output( out1, GPIO.HIGH )

        time.sleep( step_sleep )


def main():
    try:
        rotate(20)
    except KeyboardInterrupt:
        cleanup()
        exit( 1 )
 
    cleanup()
    exit( 0 )


def angle():
    pass


if __name__ == "__main__":
    main()

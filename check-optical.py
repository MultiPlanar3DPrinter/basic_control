#!/usr/bin/env python3
# Run with: ./check-optical.py | grep <0|1>
import RPi.GPIO as GPIO


in_pin = 4


def setup():
    GPIO.setmode( GPIO.BCM )
    GPIO.setup( in_pin, GPIO.IN )


def read():
    return GPIO.input(in_pin)


def main():
    setup()
    try:
        while (True):
            print(read())
    except:
        pass
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()

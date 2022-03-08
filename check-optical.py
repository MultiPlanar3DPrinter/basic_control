#!/usr/bin/env python3
# Run with: ./check-optical.py | grep <0|1>
import RPi.GPIO as GPIO


class OpticalSensor:
    def __init__(self, pin):
        self.pin = pin
        self._setup()
    
    def _setup(self):
        GPIO.setup(self.pin, GPIO.IN)
    
    def read(self):
        return GPIO.input(self.pin)


def main():
    GPIO.setmode( GPIO.BCM )
    sensor = OpticalSensor(4)
    try:
        while (True):
            print(sensor.read())
    except:
        pass
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()

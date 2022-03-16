from serial import Serial

class Printer:
    def __init__(self, port="/dev/tty.usbserial-141240", baudrate=115200):
        self._s = Serial(port=port, baudrate=baudrate, timeout=0)
        self.write(b"M155 S0")
        self.write(b"M92 X80 Y80")
        self.set_feedrate(1000)

    def write(self, cmd):
        self._s.write(cmd + b"\n")

        lines = []
        all_lines = lines
        while b"ok\n" not in b'\n'.join(lines):
            while self._s.in_waiting == 0: pass
            lines = self._s.readlines()
            all_lines += lines
        all_lines = b''.join(all_lines).splitlines()
        return '\n'.join(line.decode() for line in all_lines)

    def set_feedrate(self, amt):
        self.write(f"G0 F{amt}".encode("utf-8"))

    def move_turn(self, amt):
        self.write(f"G0 X{amt} Y{-amt}".encode("utf-8"))

    def move_tilt(self, amt):
        self.write(f"G0 X0 Y{amt}".encode("utf-8"))

    def move_turn_tilt(self, turn, tilt):
        self.write(f"G0 X{turn} Y{-turn+tilt}".encode("utf-8"))

    def set_absolute(self):
        self.write(b"G90")

    def set_relative(self):
        self.write(b"G91")
    
    def set_home(self):
        self.write(b"G92 X0 Y0")
    
    def auto_home(self):
        self.write(b"G28 X Y")


def main():
    p = Printer()

    p.set_feedrate(500)
    p.move_turn_tilt(-5, -5)

if __name__ == "__main__":
    main()

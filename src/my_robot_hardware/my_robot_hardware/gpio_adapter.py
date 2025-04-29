import platform

if platform.system() == 'Linux' and 'raspberrypi' in platform.uname().machine:
    import RPi.GPIO as GPIO
else:
    class FakeGPIO:
        BCM = 'BCM'
        OUT = 'OUT'

        def setmode(self, mode):
            print(f"[FakeGPIO] setmode({mode})")

        def setup(self, pin, mode):
            print(f"[FakeGPIO] setup(pin={pin}, mode={mode})")

        def output(self, pin, value):
            print(f"[FakeGPIO] output(pin={pin}, value={value})")

        def cleanup(self):
            print("[FakeGPIO] cleanup()")

    GPIO = FakeGPIO()

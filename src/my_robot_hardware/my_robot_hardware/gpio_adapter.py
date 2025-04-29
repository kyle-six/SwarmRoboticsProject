import platform

if platform.system() == 'Linux' and 'aarch64' in platform.uname().machine:
    # RPI running Ubuntu 20.04
    print("Running on a Raspberry Pi system, using real GPIO setup.")
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
    
    print("Running on a non-Raspberry Pi system, using mock GPIO setup.")
    GPIO = FakeGPIO()

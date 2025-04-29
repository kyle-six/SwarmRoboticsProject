import platform

if platform.system() == 'Linux' and 'raspberrypi' in platform.uname().machine:
    from gpiozero import OutputDevice
else:
    # Define a fake OutputDevice class to simulate gpiozero.OutputDevice
    class OutputDevice:
        def __init__(self, pin, active_high=True, initial_value=False):
            self.pin = pin
            self.active_high = active_high
            self.state = initial_value
            print(f"[FakeOutputDevice] Initialized on pin {pin}, active_high={active_high}, initial_value={initial_value}")

        def on(self):
            self.state = True
            print(f"[FakeOutputDevice] Pin {self.pin} set to {'HIGH' if self.active_high else 'LOW'}")

        def off(self):
            self.state = False
            print(f"[FakeOutputDevice] Pin {self.pin} set to {'LOW' if self.active_high else 'HIGH'}")

        def close(self):
            print(f"[FakeOutputDevice] Cleaning up pin {self.pin}")

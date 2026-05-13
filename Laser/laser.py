import time

from .laser_api import LaserApi


NL30X_DEVICE = b"NL30x:8"
MAXIOPG_DEVICE = b"MaxiOPG:31"
ATTENUATOR_REGISTER = b"ATTN. RotH 1.8\xb0/64 600mA."
DEFAULT_FIRE_MODE = "Continuous"
DEFAULT_SYNC_MODE = "INTERNAL"
DEFAULT_SYNC_DELAY = "-80"


class Laser:
    def __init__(self, dll_path):
        self.api = LaserApi(dll_path)
        self.dev_nl30x = NL30X_DEVICE
        self.dev_maxiopg = MAXIOPG_DEVICE
        self.connected = False
        self.initialized = False
        self.burst_count = 5

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def connect(self):
        if self.connected:
            print("Laser already connected")
            return

        self.api.connect()
        self.connected = True
        print("Laser connected")

    def disconnect(self):
        if not self.connected:
            return

        try:
            self.api.disconnect()
            print("Laser disconnected")
        finally:
            self.connected = False

    def close(self):
        self.stop_safely()
        self.disconnect()

    def stop_safely(self):
        if not self.connected:
            return

        try:
            self.stop()
        except Exception as error:
            print("Laser stop failed:", error)

    def get_string(self, device, regname, timeout_ms=2000):
        return self.api.get_string(device, regname, timeout_ms)

    def warm_up(self, seconds=10):
        self.run()
        time.sleep(seconds)
        self.stop()

    def initialize_laser(
        self,
        wavelength,
        firemode=DEFAULT_FIRE_MODE,
        syncmode=DEFAULT_SYNC_MODE,
        sync_delay=DEFAULT_SYNC_DELAY,
    ):
        if self.initialized:
            print("Laser already initialized")
            return

        print("Initializing laser...")
        self.api.set_nv_string(self.dev_nl30x, "Sync mode", syncmode)
        self.api.set_string(self.dev_nl30x, "Continuous / Burst mode / Trigger burst", firemode)
        self.api.set_string(self.dev_nl30x, "SyncOut delay", sync_delay)
        self.api.set_nv_string(self.dev_maxiopg, "Configuration", "Air")
        self.set_wavelength(wavelength)
        time.sleep(0.1)
        self.api.set_double(self.dev_maxiopg, ATTENUATOR_REGISTER, 272)
        time.sleep(1)
        self.warm_up()

        self.initialized = True
        print("Laser initialization complete")

    def set_sync_delay(self, time_val):
        self.api.set_string(self.dev_nl30x, "SyncOut delay", str(time_val))

    def set_burst_count(self, burst_count):
        if not self.initialized:
            raise RuntimeError("Please initialize the laser first")

        self.burst_count = burst_count
        self.api.set_string(self.dev_nl30x, "Burst length, pulses", str(burst_count))
        print(f"Laser burst count set to {burst_count}")

    def run(self):
        self.api.set_string(self.dev_nl30x, "State", "RUN")

    def stop(self):
        self.api.set_string(self.dev_nl30x, "State", "STOP")

    def set_wavelength(self, wavelength):
        self.api.set_nv_double(self.dev_maxiopg, "WaveLength", wavelength)
        print(f"Laser wavelength set to {wavelength} nm")


def connect_laser(dll_path):
    laser = Laser(dll_path)
    laser.connect()
    return laser

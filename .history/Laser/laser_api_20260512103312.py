import ctypes
import time
import serial.tools.list_ports


class LaserController:
    def __init__(self, dll_path):
        self.dll_path = dll_path
        self.dev_nl30x = b"NL30x:8"
        self.dev_maxiopg = b"MaxiOPG:31"
        self.rc = ctypes.windll.LoadLibrary(dll_path)
        self.connected = False
        self.initialized = False
        self.burst_count = 5  # Default burst count
        print("✅ DLL loaded successfully")

    def connect(self):
        if self.connected:
            print("🔄 Laser already connected, skipping connection step")
            return
        err = self.rc.rcConnect(1, 1)
        if err != 0:
            raise RuntimeError(f"Connection failed, error code: {err}")
        self.connected = True
        print("✅ Successfully connected to laser")

    def disconnect(self):
        try:
            self.rc.rcDisconnect()
            print("✅ Successfully disconnected from laser")
        except Exception as e:
            print("❌ Failed to disconnect:", e)

    def _set_nv_string(self, device, regname, value):
        err = self.rc.rcSetRegNVFromString(device, regname.encode(), value.encode())
        if err != 0:
            raise RuntimeError(f"Failed to set {regname} = {value}, error code: {err}")

    def _set_string(self, device, regname, value):
        err = self.rc.rcSetRegFromString(device, regname.encode(), value.encode())
        if err != 0:
            raise RuntimeError(f"Failed to set {regname} = {value}, error code: {err}")

    def _set_double(self, device, regname, val):
        if isinstance(regname, str):
            regname = regname.encode("latin1")  # According to DLL requirement
        err = self.rc.rcSetRegFromDouble(device, regname, ctypes.c_double(val))
        if err != 0:
            raise RuntimeError(f"Failed to set {regname} = {val}, error code: {err}")

    def _set_nv_double(self, device, regname, val):
        err = self.rc.rcSetRegNVFromDouble(device, regname.encode(), ctypes.c_double(val))
        if err != 0:
            raise RuntimeError(f"Failed to set {regname} = {val}, error code: {err}")

    def get_string(self, device, regname, timeout_ms=2000):
        """Read a string-type register value and its timestamp"""
        reg_val_buf = ctypes.create_string_buffer(256)
        timestamp = ctypes.c_int()

        err = self.rc.rcGetRegAsString(device, regname.encode(), reg_val_buf, 256, timeout_ms, ctypes.byref(timestamp))
        if err != 0:
            raise RuntimeError(f"Failed to read {regname}, error code: {err}")

        value = reg_val_buf.value.decode()
        return value

    def initialize_laser(self, firemode,Syncmode):
        if self.initialized:
            print("🔄 Laser alread y initialized, skipping initialization step")
            return

        print("🔧 Performing laser initialization setup...")
        self._set_nv_string(self.dev_nl30x, "Sync mode", Syncmode)
        self._set_string(self.dev_nl30x, "Continuous / Burst mode / Trigger burst", firemode)
        self._set_string(self.dev_nl30x, "SyncOut delay", "-80")
        # self._set_string(self.dev_nl30x, "Burst length, pulses", str(burst_count))
        self._set_nv_string(self.dev_maxiopg, "Configuration", "Air")
        time.sleep(0.1)
        regname = "ATTN. RotH 1.8°/64 600mA.".encode("latin1")
        self._set_double(self.dev_maxiopg, regname, 272)
        time.sleep(1)
        self._set_string(self.dev_nl30x, "State", "RUN")
        time.sleep(10)
        self._set_string(self.dev_nl30x, "State", "STOP")
        self.initialized = True
        print("✅ Initialization complete")

    def setSyncDelay(self, time_val):
        self._set_string(self.dev_nl30x, "SyncOut delay", time_val)

    def set_burts_count(self, burst_count):
        """
        Set the number of pulses in burst mode.

        Parameters:
            burst_count (int): number of pulses per burst
        """
        if not self.initialized:
            raise RuntimeError("Please initialize the laser first")
        self.burst_count = burst_count
        self._set_string(self.dev_nl30x, "Burst length, pulses", str(burst_count))
        print(f"💡 Burst count set to {burst_count}")

    def run(self):
        self._set_string(self.dev_nl30x, "State", "RUN")

    def setSync(self, time_val):
        self._set_string(self.dev_nl30x, "SyncOut delay", time_val)

    def stop(self):
        self._set_string(self.dev_nl30x, "State", "STOP")

    def setwavelength(self, wl):
        self._set_nv_double(self.dev_maxiopg, "WaveLength", wl)

    def burst_multi_wavelength(self, wavelengths):
        """
        Sequentially fire multiple wavelengths in burst mode,
        with each wavelength fired burst_count times.

        Parameters:
            wavelengths (List[int or float]): e.g., [670, 750, 850]
        """
        print(f"🚀 Multi-wavelength burst: {wavelengths}, {self.burst_count} pulses each")

        # Initialize laser state
        self._set_string(self.dev_nl30x, "State", "RUN")
        time.sleep(5)  # Laser warm-up time

        for wl in wavelengths:
            self._set_nv_double(self.dev_maxiopg, "WaveLength", wl)
            self._set_string(self.dev_nl30x, "Continuous / Burst mode / Trigger burst", "Trigger")
            print(f"💡 Triggered burst at {wl} nm")
            time.sleep(self.burst_count * 0.1)  # Estimated emission delay

        self._set_string(self.dev_nl30x, "State", "STOP")
        print("✅ All wavelength bursts completed")


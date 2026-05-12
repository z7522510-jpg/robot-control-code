import ctypes
import os


class LaserApi:
    def __init__(self, dll_path):
        self.dll_path = os.path.abspath(dll_path)
        self._dll_directory = None
        if hasattr(os, "add_dll_directory"):
            self._dll_directory = os.add_dll_directory(os.path.dirname(self.dll_path))
        self.rc = ctypes.windll.LoadLibrary(self.dll_path)
        print("Laser DLL loaded:", self.dll_path)

    def _check(self, err, message):
        if err != 0:
            raise RuntimeError(f"{message}, error code: {err}")

    def connect(self, connection_type=1, comport_number=1):
        err = self.rc.rcConnect(connection_type, comport_number)
        self._check(err, "Laser connection failed")

    def disconnect(self):
        err = self.rc.rcDisconnect()
        self._check(err, "Laser disconnect failed")

    def set_nv_string(self, device, regname, value):
        err = self.rc.rcSetRegNVFromString(device, regname.encode(), value.encode())
        self._check(err, f"Failed to set {regname} = {value}")

    def set_string(self, device, regname, value):
        err = self.rc.rcSetRegFromString(device, regname.encode(), value.encode())
        self._check(err, f"Failed to set {regname} = {value}")

    def set_double(self, device, regname, value):
        if isinstance(regname, str):
            regname = regname.encode("latin1")
        err = self.rc.rcSetRegFromDouble(device, regname, ctypes.c_double(value))
        self._check(err, f"Failed to set {regname} = {value}")

    def set_nv_double(self, device, regname, value):
        err = self.rc.rcSetRegNVFromDouble(device, regname.encode(), ctypes.c_double(value))
        self._check(err, f"Failed to set {regname} = {value}")

    def get_string(self, device, regname, timeout_ms=2000):
        reg_val_buf = ctypes.create_string_buffer(256)
        timestamp = ctypes.c_int()

        err = self.rc.rcGetRegAsString(
            device,
            regname.encode(),
            reg_val_buf,
            256,
            timeout_ms,
            ctypes.byref(timestamp),
        )
        self._check(err, f"Failed to read {regname}")

        return reg_val_buf.value.decode()

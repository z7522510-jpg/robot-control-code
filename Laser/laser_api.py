import ctypes
import os


class LaserApi:
    def __init__(self, dll_path):
        self.dll_path = os.path.abspath(dll_path)
        self.dll_dir = os.path.dirname(self.dll_path)
        self.rc = ctypes.windll.LoadLibrary(self.dll_path)
        print("DLL loaded successfully")

    def connect(self):
        original_cwd = os.getcwd()
        try:
            os.chdir(self.dll_dir)
            err = self.rc.rcConnect(1, 1)
        finally:
            os.chdir(original_cwd)
        if err != 0:
            raise RuntimeError(f"Connection failed, error code: {err}")

    def disconnect(self):
        self.rc.rcDisconnect()

    def set_nv_string(self, device, regname, value):
        err = self.rc.rcSetRegNVFromString(device, regname.encode(), value.encode())
        if err != 0:
            raise RuntimeError(f"Failed to set {regname} = {value}, error code: {err}")

    def set_string(self, device, regname, value):
        err = self.rc.rcSetRegFromString(device, regname.encode(), value.encode())
        if err != 0:
            raise RuntimeError(f"Failed to set {regname} = {value}, error code: {err}")

    def set_double(self, device, regname, val):
        if isinstance(regname, str):
            regname = regname.encode("latin1")
        err = self.rc.rcSetRegFromDouble(device, regname, ctypes.c_double(val))
        if err != 0:
            raise RuntimeError(f"Failed to set {regname} = {val}, error code: {err}")

    def set_nv_double(self, device, regname, val):
        err = self.rc.rcSetRegNVFromDouble(device, regname.encode(), ctypes.c_double(val))
        if err != 0:
            raise RuntimeError(f"Failed to set {regname} = {val}, error code: {err}")

    def get_string(self, device, regname, timeout_ms=2000):
        reg_val_buf = ctypes.create_string_buffer(256)
        timestamp = ctypes.c_int()

        err = self.rc.rcGetRegAsString(
            device, regname.encode(), reg_val_buf, 256, timeout_ms, ctypes.byref(timestamp)
        )
        if err != 0:
            raise RuntimeError(f"Failed to read {regname}, error code: {err}")

        return reg_val_buf.value.decode()

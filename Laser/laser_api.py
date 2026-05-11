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

    def connect(self, connection_type=1, comport_number=1):
        return self.rc.rcConnect(connection_type, comport_number)

    def disconnect(self):
        return self.rc.rcDisconnect()

    def set_reg_nv_string(self, device, regname, value):
        return self.rc.rcSetRegNVFromString(device, regname.encode(), value.encode())

    def set_reg_string(self, device, regname, value):
        return self.rc.rcSetRegFromString(device, regname.encode(), value.encode())

    def set_reg_double(self, device, regname, value):
        if isinstance(regname, str):
            regname = regname.encode("latin1")
        return self.rc.rcSetRegFromDouble(device, regname, ctypes.c_double(value))

    def set_reg_nv_double(self, device, regname, value):
        return self.rc.rcSetRegNVFromDouble(device, regname.encode(), ctypes.c_double(value))

    def get_reg_string(self, device, regname, timeout_ms=2000):
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
        if err != 0:
            return err, None

        return err, reg_val_buf.value.decode()

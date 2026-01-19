import math
import serial

class RadarMR76():
    def __init__(self, address="/dev/ttyUSB0", baudrate=115200, timeout=1):
        self.cur_len_obj = 0
        self.tot_det_obj = 0
        self.obj_list = []

        self.port = serial.Serial(address, baudrate=baudrate, timeout=timeout)
        print("MR76 connected on", address)

    def read(self):
        data = []
        app = False
        decode = False
        complete_packet = False
        max_bytes = 200
        count = 0

        while not complete_packet and count < max_bytes:
            x = self.port.read()
            count += 1

            if x == b't':
                app = True
                decode = False
                data = []
                continue

            elif x == b'\r':
                decode = True
                app = False

            if app:
                data.append(x)

            if decode and len(data) == 20:
                return b''.join(data).decode("utf-8")

        return None

    def parse_info(self, data):
        if isinstance(data, str):
            data = bytes.fromhex(data)

        return [
            data[2],
            (data[3] << 8) | data[4]
        ]

    def parse_target_info(self, data):
        if isinstance(data, str):
            data = bytes.fromhex(data)

        ds = [0] * 8
        ds[0] = data[2]
        ds[1] = ((data[3] << 8) | (data[4] & 0xF8)) >> 3
        ds[2] = ((data[4] & 0x07) << 8) | data[5]
        ds[3] = ((data[6] << 8) | (data[7] & 0xC0)) >> 6
        ds[4] = (((data[7] & 0x3F) << 8) | (data[8] & 0xE0)) >> 5
        ds[6] = data[8] & 0x07
        ds[7] = data[9]

        dist_long = ds[1] * 0.2 - 500
        dist_lat  = ds[2] * 0.2 - 204.6

        return {
            'id': ds[0],
            'dist_long': dist_long,
            'dist_lat': dist_lat,
            'dynamic_prop': ds[6],
            'rcs': ds[7] * 0.5 - 64.0
        }

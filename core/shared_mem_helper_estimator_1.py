import struct
import time


class helper:
    def __init__(self, shm, keys):
        self.shm = shm
        self.keys = keys
        self.imu_updates = 0
        self.gps_updates = 0
        self.xh_updates = 0

    def read_y(self, use_calibrated_imu_data=True):
        indexes = []
        for key in self.keys.keys():
            if "ADC" in key:
                continue  # not really important for estimator
            if key.startswith("y_"):
                if use_calibrated_imu_data and "RAW" in key:
                    continue
                if not use_calibrated_imu_data and "CALIB" in key:
                    continue
                indexes.append(self.keys[key])
        while True:
            y = [struct.unpack("d", self.shm.read(8, index * 8))[0] for index in indexes]
            if y[0] == self.imu_updates:
                time.sleep(0.001)  # wait if no new data
                continue
            self.imu_updates = y[0]
            if self.gps_updates == y[1]:
                new_gps = False
            else:
                self.gps_updates = y[1]
                new_gps = True
            y.insert(2, new_gps)
            return y[2:]

    def read_xh(self, xh_index=1):
        xh_prefix = f"xh_{xh_index}_"
        indexes = [self.keys[key] for key in self.keys.keys() if key.startswith(xh_prefix)]
        xh = [struct.unpack("d", self.shm.read(8, index * 8))[0] for index in indexes]
        return xh[1:]

    def write_xh(self, values):
        indexes = [self.keys[key] for key in self.keys.keys() if key.startswith("xh_1_")]
        self.xh_updates += 1
        values.insert(0, self.xh_updates)
        if len(values) != len(indexes):
            print(
                f"ERROR [ESTIMATOR_1]: Invalid number of values provided to write to xh_1 vector.\nExpected {len(indexes)} values, but received {len(values)}."
            )
            return
        for i in range(len(indexes)):
            self.shm.write(struct.pack("d", values[i]), indexes[i] * 8)

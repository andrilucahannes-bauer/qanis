from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from ahrs.filters import Madgwick
import numpy as np


class IMUController:
    def __init__(self, sample_freq=256.0, beta=0.1):
        self.mpu = MPU9250(
            address_ak=AK8963_ADDRESS,
            address_mpu_master=MPU9050_ADDRESS_68,
            address_mpu_slave=None,
            bus=1,
            gfs=GFS_1000,
            afs=AFS_8G,
            mfs=AK8963_BIT_16,
            mode=AK8963_MODE_C100HZ,
        )
        self.mpu.configure()

        self.filter = Madgwick(frequency=sample_freq, gain=beta)
        self.q      = [1.0, 0.0, 0.0, 0.0]   # initial orientation

    def get_euler(self):
        accel = self.mpu.readAccelerometerMaster()
        gyro  = [np.radians(g) for g in self.mpu.readGyroscopeMaster()]

        # 1) update quaternion in place
        self.q = self.filter.updateIMU(q=self.q,
                                       gyr=gyro,
                                       acc=accel,)

        # 2) convert q → roll, pitch, yaw
        qw, qx, qy, qz = self.q
        roll  = np.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = np.asin( max(-1, min(1, 2*(qw*qy - qz*qx))) )
        yaw   = np.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

        #print(f"roll: {np.rad2deg(roll)}, pitch: {np.rad2deg(pitch)}")

        return (roll, pitch, yaw)

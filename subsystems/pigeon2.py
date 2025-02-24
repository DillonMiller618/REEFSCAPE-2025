from subsystems.gyrobase import GyroBase
from constants import DriveConstants
from wpimath.geometry import Rotation2d

from phoenix6.hardware.pigeon2 import Pigeon2

class Pigeon(GyroBase):
    def __init__(self):
        self.gyro = Pigeon2(DriveConstants.kGryoCanID)
        #TODO: add invert code

    def get_angle(self):
        return self.gyro.getRotation2d()
    
    def reset(self, new_angle: Rotation2d = Rotation2d.fromDegrees(0)) -> None:
        if new_angle == Rotation2d.fromDegrees(0):
            self.gyro.reset()
        self.gyro.set_yaw(new_angle.degrees())
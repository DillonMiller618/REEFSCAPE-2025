from subsystems.arm2 import Arm
import commands2
from rev import SparkMax, SparkBase
from wpimath.geometry import Rotation2d

class ArmMove(commands2.command):
    def __init__(self, arm: Arm, angle: float, toleranceInches=0.5):
        super().__init__()

        # position must be callable
        self.position = Rotation2d.fromDegrees(angle)
        self.arm = arm

    def initialize(self):
        pass

    def isFinished(self) -> bool:
        pass

    def end(self, interrupted: bool):
        self.arm.stopMotor()

    def execute(self):
        self.arm.setAngleGoal(angle=self.position)
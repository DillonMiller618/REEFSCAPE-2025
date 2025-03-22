from subsystems.arm2 import Arm
import commands2
from rev import SparkMax, SparkBase
from wpimath.geometry import Rotation2d

class ArmMove(commands2.Command):
    def __init__(self, arm: Arm, angle_or_Speed: float, useSpeedControl=False):
        """
        Moves the arm based on a parameter angle, which can be turned into speed with the parameters"""
        super().__init__()

        self.useSpeedControl = useSpeedControl
        self.arm = arm
        self.control = angle_or_Speed
        

    def initialize(self):
        pass

    def isFinished(self) -> bool:
        pass

    def end(self, interrupted: bool):
        if self.useSpeedControl:
            self.arm.stopMotor()
        else:
            self.arm.setAngleGoal(self.arm.getPosition()) #effectively stops the motor

    def execute(self):
        if self.useSpeedControl:
            self.arm.setSpeed(self.control)
        else:
            self.arm.setAngleGoal(self.control)
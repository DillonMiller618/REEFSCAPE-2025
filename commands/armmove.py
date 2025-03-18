from subsystems.arm import Arm
import commands2
from rev import SparkMax, SparkBase

class ArmMove(commands2.command):
    def __init__(self, arm: Arm, position: float, toleranceInches=0.5):
        super().__init__()

        # position must be callable
        self.position = position
        self.pivmotor = SparkMax(1)

        self.controller = self.pivmotor.getClosedLoopController()
        self.controller.setReference(self.position, SparkBase.ControlType.kPosition)

    def initialize(self):
        pass

    def isFinished(self) -> bool:
        pass

    def end(self, interrupted: bool):
        pass

    def execute(self):
        pass
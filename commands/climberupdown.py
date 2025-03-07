import commands2
from subsystems.climber import Climber
from constants import ELEC


#TODO: Code is untested
class ClimberMove(commands2.Command):
    def __init__(self, speed: float):
        self.speed = speed
        """
        Moves the climber at set constant rates. 1 is forward, -1 is backward
        """
    
    def initialize(self):
        self.climber = Climber(ELEC.Climber_CAN_ID)

    def isFinished(self):
        return True
    
    def execute(self):
        self.climber.moveClimber(self.speed)
    
    def end(self, interrupted: bool):
        ...
        #Nothing to put here
import commands2
from subsystems.climber import Climber
from constants import ELEC


#TODO: Code is untested
class ClimberMove(commands2.Command):
    def __init__(self, speed: float, climber: Climber):
        self.climber = climber
        self.speed = speed
        self.addRequirements(climber)
        """
        Moves the climber at set constant rates. 1 is forward, -1 is backward
        """
    
    def initialize(self):
        pass

    def isFinished(self) -> bool:
        return False
    
    def execute(self):
        self.climber.moveClimber(self.speed)
    
    def end(self, interrupted: bool):
        self.climber.stopClimber()
        #Nothing to put here
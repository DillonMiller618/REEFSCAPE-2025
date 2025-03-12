import commands2
from subsystems.climber import Climber
from subsystems.shooter import Shooter
from constants import ELEC


#TODO: Test code on robot
#TODO: Whenever this command is run, move arm to climbersetpoint
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
        #add code to set arm to setpoint

    def isFinished(self) -> bool:
        return False
    
    def execute(self):
        self.climber.moveClimber(self.speed)
    
    def end(self, interrupted: bool):
        self.climber.stopClimber()


#shooter commands
class Shoot(commands2.Command):
    def __init__(self, speed: float, shooter: Shooter):
        self.shooter = shooter
        self.speed = speed
        self.addRequirements(shooter)
        """
        Moves the climber at set constant rates. 1 is forward, -1 is backward
        """
    
    def initialize(self):
        pass

    def isFinished(self) -> bool:
        return False
    
    def execute(self):
        self.shooter.spinShooter(self.speed)
    
    def end(self, interrupted: bool):
        self.shooter.stopShooter()
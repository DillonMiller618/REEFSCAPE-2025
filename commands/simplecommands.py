import commands2
from subsystems.climber import Climber
from subsystems.shooter import Shooter
from subsystems.arm import Arm
from subsystems.flipper import Flipper
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


class ArmMove(commands2.Command):
    def __init__(self, speed: float, arm: Arm):
        """Deprecated, do not use for speed, instead use armmove.py"""
        self.arm = arm
        self.speed = speed
        self.addRequirements(arm)
        """
        Moves the climber at set constant rates. 1 is forward, -1 is backward
        """
    
    def initialize(self):
        pass
        #add code to set arm to setpoint

    def isFinished(self) -> bool:
        return False
    
    def execute(self):
        self.arm.startArm(self.speed)
    
    def end(self, interrupted: bool):
        self.arm.stopAndReset()

#shooter commands
class Shoot(commands2.Command):
    def __init__(self, speed: float, shooter: Shooter):
        self.shooter = shooter
        self.speed = speed
        self.addRequirements(shooter)
        """
        Moves the shoot at set constant rates. 1 is forward, -1 is backward
        """
    
    def initialize(self):
        pass

    def isFinished(self) -> bool:
        return False
    
    def execute(self):
        self.shooter.spinShooter(self.speed)
    
    def end(self, interrupted: bool):
        self.shooter.stopShooter()
        

class FlipCoral(commands2.Command):
    def __init__(self, flipper: Flipper, speed: float):
        self.flipper = flipper
        self.speed = speed
        self.addRequirements(flipper)

    def initialize(self):
        pass

    def isFinished(self) -> bool:
        return False
    
    def execute(self):
        #self.time.restart()
        self.flipper.flip(self.speed)
    
    def end(self, interrupted: bool):
        self.flipper.stopMotor()
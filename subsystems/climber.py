from rev import SparkMax, SparkMaxConfig, SparkBase
from commands2 import subsystem

class ClimberConstants:
    gearRatio = 1
    kP = .1
    kI = 0.0
    kD = 0.0
    kPositionConversionFactor = 1.0
    kVelocityConversionFactor = 1.0
    #determines max speed of turning motor
    kMaxPower = 0.5 # max of 1.0

class Climber(subsystem):
    def __init__(self, motorCanID: int):
        super().__init__()

        # Start class and config, get constants
        constants = ClimberConstants
        self.motor = SparkMax(motorCanID, SparkBase.MotorType.kBrushless)
        self.mconfig = SparkMaxConfig()
        # Configure the motor with constants
        self.mconfig.inverted(False)
        self.mconfig.setIdleMode(SparkBase.IdleMode.kBrake)
        self.mconfig.encoder.positionConversionFactor(constants.kPositionConversionFactor)
        self.mconfig.encoder.velocityConversionFactor(constants.kVelocityConversionFactor)
        #self.mconfig.closedLoop.pid(constants.kP, constants.kI, constants.kD)
        #self.mconfig.closedLoop.velocityFF(0.0)
        self.motor.configure(self.mconfig)

    def moveClimber(self, speed):
        self.motor.set(speed * ClimberConstants.kMaxPower)
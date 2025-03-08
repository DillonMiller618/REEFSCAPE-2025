from rev import SparkBaseConfig, SparkMax, SparkMaxConfig, SparkBase
from commands2 import Subsystem
from constants import ELEC

class ClimberConstants:
    gearRatio = 1

    kPositionConversionFactor = 1.0
    kVelocityConversionFactor = 1.0
    #determines max speed of turning motor
    kMaxPower = 0.5 # max of 1.0

class Climber(Subsystem):
    def __init__(self, motorCanID: int):
        super().__init__()

        # Start class and config, get constants
        constants = ClimberConstants
        self.motor = SparkMax(motorCanID, SparkBase.MotorType.kBrushless)
        self.mconfig = SparkBaseConfig()
        # Configure the motor with constants
        self.mconfig.inverted(False)
        self.mconfig.smartCurrentLimit(ELEC.climber_continuous_current_limit)
        self.mconfig.secondaryCurrentLimit(ELEC.climber_peak_current_limit)

        self.mconfig.openLoopRampRate(ELEC.open_loop_ramp_rate)
        self.mconfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        self.mconfig.encoder.positionConversionFactor(constants.kPositionConversionFactor)
        self.mconfig.encoder.velocityConversionFactor(constants.kVelocityConversionFactor)
        self.motor.configure(self.mconfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

    def moveClimber(self, speed):
        self.motor.set(speed * ClimberConstants.kMaxPower)

    def stopClimber(self):
        self.motor.stopMotor()
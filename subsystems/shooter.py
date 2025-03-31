from rev import SparkBaseConfig, SparkMax, SparkMaxConfig, SparkBase
from commands2 import Subsystem
from constants import ELEC
from utils import getLeadMotorConfig

class ShooterConstants:
    gearRatio = 1
    invert = False

    kPositionConversionFactor = 1.0
    kVelocityConversionFactor = 1.0
    #determines max speed of turning motor
    kMaxPower = 0.75 # max of 1.0
    kTimeAfterDetect = 0.1 # time interval between first detect of coral entering shooter

    #Ultrasonic sensor constants

class Shooter(Subsystem):
    def __init__(self, motorCanID: int, followMotorCANID: int | None):
        super().__init__()

        if followMotorCANID is not None:
            self.isfollowMotor = True
        # Start class and config, get constants
        constants = ShooterConstants
        self.motor = SparkMax(motorCanID, SparkBase.MotorType.kBrushless) 
               
        self.mconfig = SparkBaseConfig()
        # Configure the motor with constants
        self.mconfig.inverted(False)
        self.mconfig.smartCurrentLimit(ELEC.vortex_continuous_current_limit)
        self.mconfig.secondaryCurrentLimit(ELEC.vortex_peak_current_limit)

        self.mconfig.openLoopRampRate(ELEC.open_loop_ramp_rate)
        self.mconfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        self.mconfig.encoder.positionConversionFactor(constants.kPositionConversionFactor)
        self.mconfig.encoder.velocityConversionFactor(constants.kVelocityConversionFactor)
        self.motor.configure(self.mconfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        if self.isfollowMotor:
            self.followmotor = SparkMax(followMotorCANID, SparkBase.MotorType.kBrushless)
            self.m2config = SparkMaxConfig()
            self.mconfig.smartCurrentLimit(ELEC.vortex_continuous_current_limit)
            self.mconfig.secondaryCurrentLimit(ELEC.vortex_peak_current_limit)
            self.mconfig.openLoopRampRate(ELEC.open_loop_ramp_rate)
            self.mconfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
            self.mconfig.encoder.positionConversionFactor(constants.kPositionConversionFactor)
            self.mconfig.encoder.velocityConversionFactor(constants.kVelocityConversionFactor)
            self.motor.configure(self.mconfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
            self.m2config.follow(motorCanID, invert=True)
            self.followmotor.configure(self.m2config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

    def spinShooter(self, speed):
        self.motor.set(speed * ShooterConstants.kMaxPower)
        if self.isfollowMotor:
            self.followmotor.set(min(speed, ShooterConstants.kMaxPower))

    def stopShooter(self):
        self.motor.stopMotor()        
        if self.isfollowMotor:
            self.followmotor.stopMotor()
            
    """
    #deprecated for the time being
    def feedShooter(self):
        pass
        self.currentdist = self.ultrasonic.getValue() * self.voltage_scale_factor * 0.0492 #in inches
    """
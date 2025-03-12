from wpilib import AnalogInput, RobotController
from rev import SparkBaseConfig, SparkMax, SparkMaxConfig, SparkBase
from commands2 import Subsystem
from constants import ELEC

class ShooterConstants:
    gearRatio = 1

    kPositionConversionFactor = 1.0
    kVelocityConversionFactor = 1.0
    #determines max speed of turning motor
    kMaxPower = 0.75 # max of 1.0
    kTimeAfterDetect = 0.1 # time interval between first detect of coral entering shooter

    #Ultrasonic sensor constants

class Shooter(Subsystem):
    def __init__(self, motorCanID: int):
        super().__init__()

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

        self.ultrasonic = AnalogInput(0)
        self.voltage_scale_factor = 5/RobotController.getVoltage5V()
        self.normaldist = self.ultrasonic.getValue() * self.voltage_scale_factor * 0.0492 #in inches

    def spinShooter(self, speed):
        self.motor.set(speed * ShooterConstants.kMaxPower)

    def stopShooter(self):
        self.motor.stopMotor()

    def feedShooter(self):
        pass
        self.currentdist = self.ultrasonic.getValue() * self.voltage_scale_factor * 0.0492 #in inches
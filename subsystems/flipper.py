from rev import SparkMax, SparkBase, SparkBaseConfig, ClosedLoopConfig, SparkMaxConfig, ClosedLoopSlot
from commands2 import Subsystem
from constants import ELEC


class FlipperConstants:
    kEncoderInverted = False
    kEncoderPositionConversionFactor = 1
    kEncoderVelocityConversionFactor = kEncoderPositionConversionFactor / 60
    kGearReduction = 16.0

class Flipper(Subsystem):
    def __init__(self, leadMotorCanID: int):
        super().__init__()
        constants = FlipperConstants
        self.motor = SparkMax(leadMotorCanID, SparkBase.MotorType.kBrushless)
        self.mconfig = SparkBaseConfig()
        # Configure the motor with constants
        self.mconfig.inverted(constants.kEncoderInverted)
        self.mconfig.smartCurrentLimit(ELEC.vortex_continuous_current_limit)
        self.mconfig.secondaryCurrentLimit(ELEC.vortex_peak_current_limit)

        self.mconfig.openLoopRampRate(ELEC.open_loop_ramp_rate)
        self.mconfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        self.mconfig.encoder.positionConversionFactor(constants.kEncoderPositionConversionFactor)
        self.mconfig.encoder.velocityConversionFactor(constants.kEncoderVelocityConversionFactor)
        self.motor.configure(self.mconfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        #TODO: PID Control for bonus points
    
    def flip(self, speed: float):
        """Use negative speed to flip down, positive speed to flip up"""
        self.motor.set(speed)

    def stopMotor(self):
        """Just to make sure the motor doesn't get any funny ideas."""
        self.motor.stopMotor()
        
        


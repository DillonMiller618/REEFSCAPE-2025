from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from rev import SparkMax, SparkMaxConfig, SparkMaxConfigAccessor, SparkBaseConfig
from phoenix5.sensors import CANCoder, AbsoluteSensorRange, CANCoderStatusFrame, CANCoderConfiguration
from constants import DriveConstants, ModuleConstants
import math
from typing import Union

from wpilib import SmartDashboard


class SwerveModule:

    def __init__(self, dm_id: int, sm_id: int, enc_id: int, mod_offset: float, drive_invert: bool, steer_invert: bool):
        """
        dm_id: Drive motor CAN ID, int.
        sm_id: Steer motor CAN ID, int.
        enc_id: Absolute encoder CAN ID, int.
        mod_offset: For Phoenix5 CANCoder setup, float. <- Scheduled to be deprecated.
        drive_invert: Boolean for inverting drive motor, True/False.
        steer_invert: Boolean for inverting steer motor, True/False.
        """
        # Connect the passed items to default variables.
        self.drive_motor = SparkMax(dm_id, SparkMax.MotorType.kBrushless)
        self.steer_motor = SparkMax(sm_id, SparkMax.MotorType.kBrushless)
        self.encoder = CANCoder(enc_id, "rio")
        self.drive_encoder = self.drive_motor.getEncoder()


        # Configure the Driving Spark Max motor
        dconfig = SparkMaxConfig()
        dconfig.inverted(drive_invert)
        dconfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        dconfig.encoder.positionConversionFactor(DriveConstants.dPositionConversionFactor)
        dconfig.encoder.velocityConversionFactor(DriveConstants.dVelocityConversionFactor)
        dconfig.closedLoop.FeedbackSensor(dconfig.closedLoop.FeedbackSensor.kPrimaryEncoder)
        dconfig.closedLoop.pid(DriveConstants.ob_drive_pid[0],
                              DriveConstants.ob_drive_pid[1],
                              DriveConstants.ob_drive_pid[2])
        dconfig.closedLoopRampRate(DriveConstants.closed_loop_ramp)
        dconfig.openLoopRampRate(DriveConstants.open_loop_ramp)
        dconfig.smartCurrentLimit(DriveConstants.drive_current_limit)
        

        # Configure the Turning Spark Max Motor
        tconfig = SparkMaxConfig()
        tconfig.inverted(steer_invert)
        tconfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        tconfig.encoder.positionConversionFactor(DriveConstants.dPositionConversionFactor)
        tconfig.encoder.velocityConversionFactor(DriveConstants.dVelocityConversionFactor)
        tconfig.encoder.positionConversionFactor(1 / 21.42857)
        tconfig.closedLoop.FeedbackSensor(tconfig.closedLoop.FeedbackSensor.kPrimaryEncoder)
        tconfig.closedLoop.pid(DriveConstants.ob_steer_pid[0],
                                DriveConstants.ob_steer_pid[1],
                                DriveConstants.ob_steer_pid[2])
        tconfig.closedLoopRampRate(DriveConstants.closed_loop_ramp)
        tconfig.openLoopRampRate(DriveConstants.open_loop_ramp)
        tconfig.smartCurrentLimit(DriveConstants.azimuth_current_limit)


        self.steer_pid = self.drive_motor.getClosedLoopController()
        self.drive_pid = self.steer_motor.getClosedLoopController()


        # Burn onto the flash
        self.drive_motor.configure(dconfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters)
        self.steer_motor.configure(tconfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters)

        # Cancoder Config
        self.encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20)
        self.encoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 20)
        self.encoder.setPositionToAbsolute()
        self.encoder.configMagnetOffset(mod_offset)
        self.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.encoder.configSensorDirection(ModuleConstants.kTurningEncoderInverted)

        # Get encoder and start with the cancoder values
        self.steer_enc = self.steer_motor.getEncoder()
        self.set_relative_start()

    def degree_to_steer(self, angle: Rotation2d) -> float:
        """Convert a value from 0 to 360 to a value from 0 to 1."""
        return angle.radians() / (2 * math.pi)

    def set_desired_state_onboard(self, desired_state: SwerveModuleState):
        """Set a desired swerve module state for SPARK MAXes."""
        state = self.optimize_onboard(desired_state)

        if 0 < self.degree_to_steer(state.angle) <= 0.25 and 0.75 <= self.steer_enc.getPosition() % 1 < 1:
            wrap_add = 1
        elif 0 < self.steer_enc.getPosition() % 1 <= 0.25 and 0.75 <= self.degree_to_steer(state.angle) < 1:
            wrap_add = -1
        else:
            wrap_add = 0

        angle_mod = self.degree_to_steer(state.angle) + math.trunc(self.steer_enc.getPosition()) + wrap_add
        angle_mod /= 5
        self.drive_pid.setReference(state.speed, SparkMax.ControlType.kVelocity)
        self.steer_pid.setReference(angle_mod, SparkMax.ControlType.kPosition)

        SmartDashboard.putNumber("anglemod", angle_mod)

    def reset_encoders(self):
        """Reset the drive encoder to its zero position."""
        self.drive_encoder.setPosition(0)

    def set_relative_start(self):
        """Preset the relative encoder on steering SPARK MAXes."""
        signed180input = self.encoder.getAbsolutePosition()
        if signed180input < 0:
            signed180input += 360

        self.steer_enc.setPosition((signed180input / 360) + 5)

    def get_current_draw(self) -> Union[float, float]:
        """Returns a list of the drive and steering motor current draws."""
        return [self.drive_motor.getOutputCurrent(), self.steer_motor.getOutputCurrent()]

    def get_state_onboard(self) -> SwerveModuleState:
        """Returns the current swerve module state."""
        return SwerveModuleState(self.drive_encoder.getVelocity(),
                                 Rotation2d((self.steer_enc.getPosition() % 1) * math.pi * 2))

    def get_position_onboard(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drive_encoder.getPosition(),
                                    Rotation2d((self.steer_enc.getPosition() % 1) * math.pi * 2))

    def optimize_onboard(self, desired_state: SwerveModuleState):
        inverted = False
        desired_degrees = desired_state.angle.degrees()
        if desired_degrees < 0:
            desired_degrees += 360  # converts desired degrees to 360

        current_degrees = (self.steer_enc.getPosition() % 1) * 360  # converts current to 360

        if 90.0 < abs(current_degrees - desired_degrees) <= 270.0:
            inverted = True
            if desired_degrees > 180:
                desired_degrees -= 180
            else:
                desired_degrees += 180

        magnitude = desired_state.speed

        if inverted:
            magnitude *= -1

        return SwerveModuleState(magnitude, Rotation2d.fromDegrees(desired_degrees))
import math
from typing import Union

from wpimath.units import degreesToRadians
from rev import SparkMax, SparkLowLevel, SparkBase
from wpimath.geometry import Rotation2d
from wpilib import DriverStation
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
#from phoenix6.hardware.cancoder import CANcoder
from phoenix5.sensors import CANCoder, CANCoderStatusFrame, AbsoluteSensorRange
from phoenix5.sensors import CANCoderConfiguration

from constants import ModuleConstants, getSwerveDrivingMotorConfig, getSwerveTurningMotorConfig

class SwerveModule:
    def __init__(
        self,
        drivingCANId: int,
        turningCANId: int,
        encoderCANId: int,
        chassisAngularOffset: float,
        turnMotorInverted: True,
        motorControllerType = SparkMax
    ) -> None:
        """Constructs a MAXSwerveModule and configures the driving and turning motor,
        encoder, and PID controller. This configuration is specific to the REV
        MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
        Encoder.
        """
        self.chassisAngularOffset = 0
        self.desiredState = SwerveModuleState(0.0, Rotation2d())


        # Declares each motor as brushless, sparkmaxes (see above), and gets canid from constants file
        self.drivingSparkMax = motorControllerType(
            drivingCANId, SparkLowLevel.MotorType.kBrushless
        )
        self.turningSparkMax = motorControllerType(
            turningCANId, SparkLowLevel.MotorType.kBrushless
        )
        self.encoder = CANCoder(encoderCANId)
        self.drivingEncoder = self.drivingSparkMax.getEncoder()

        # Factory reset, so we get the SPARKS MAX to a known state before configuring
        # them. This is useful in case a SPARK MAX is swapped out.
        self.drivingSparkMax.configure(
            getSwerveDrivingMotorConfig(),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        self.turningSparkMax.configure(
            getSwerveTurningMotorConfig(turnMotorInverted),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        self.encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20)
        self.encoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 20)
        self.encoder.setPositionToAbsolute()
        self.encoder.configMagnetOffset(0)
        #TODO check this later
        self.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)

        #Invert motors based on instantiation
        self.drivingSparkMax.setInverted(True)
        self.turningSparkMax.setInverted(False)

        self.steeringEncoder = self.turningSparkMax.getEncoder()
        self.steeringEncoder.setPosition(0)

        self.drivingPIDController = self.drivingSparkMax.getClosedLoopController()
        self.turningPIDController = self.turningSparkMax.getClosedLoopController()


        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState.angle = Rotation2d(degreesToRadians(self.encoder.getAbsolutePosition())) #idk abt this
        self.drivingEncoder.setPosition(0)

    def degree_to_steer(self, angle: Rotation2d) -> float:
        """Convert a value from 0 to 360 to a value from 0 to 1."""
        return angle.radians() / (2 * math.pi)

    def set_desired_state_onboard(self, desired_state: SwerveModuleState):
        """Set a desired swerve module state for SPARK MAXes."""
        state = self.optimize_onboard(desired_state)

        if 0 < self.degree_to_steer(state.angle) <= 0.25 and 0.75 <= self.steeringEncoder.getPosition() % 1 < 1:
            wrap_add = 1
        elif 0 < self.steeringEncoder.getPosition() % 1 <= 0.25 and 0.75 <= self.degree_to_steer(state.angle) < 1:
            wrap_add = -1
        else:
            wrap_add = 0

        angle_mod = self.degree_to_steer(state.angle) + math.trunc(self.steeringEncoder.getPosition()) + wrap_add
        self.drivingPIDController.setReference(state.speed, SparkMax.ControlType.kVelocity)
        self.turningPIDController.setReference(angle_mod, SparkMax.ControlType.kPosition)

    def reset_encoders(self):
        """Reset the drive encoder to its zero position."""
        self.drivingEncoder.setPosition(0)

    def set_relative_start(self):
        """Preset the relative encoder on steering SPARK MAXes."""
        signed180input = self.encoder.getAbsolutePosition()
        if signed180input < 0:
            signed180input += 360

        self.steeringEncoder.setPosition((signed180input / 360) + 5)

    def get_current_draw(self) -> Union[float, float]:
        """Returns a list of the drive and steering motor current draws."""
        return [self.drivingSparkMax.getOutputCurrent(), self.turningSparkMax.getOutputCurrent()]

    def get_state_onboard(self) -> SwerveModuleState:
        """Returns the current swerve module state."""
        return SwerveModuleState(self.drivingEncoder.getVelocity(),
                                 Rotation2d((self.steeringEncoder.getPosition() % 1) * math.pi * 2))

    def get_position_onboard(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drivingEncoder.getPosition(),
                                    Rotation2d((self.steeringEncoder.getPosition() % 1) * math.pi * 2))

    def optimize_onboard(self, desired_state: SwerveModuleState):
        inverted = False
        desired_degrees = desired_state.angle.degrees()
        if desired_degrees < 0:
            desired_degrees += 360  # converts desired degrees to 360

        current_degrees = (self.steeringEncoder.getPosition() % 1) * 360  # converts current to 360

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
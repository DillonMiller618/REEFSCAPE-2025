import enum
import math

import phoenix5.sensors
import phoenix6.hardware
import rev
import wpilib
from wpimath.geometry import Rotation2d

from ..abstract.sensor import AbsoluteEncoder, Gyro


class AbsoluteCANCoder(AbsoluteEncoder):
    def __init__(self, id_: int | tuple[int, str], invert: bool = False):
        super().__init__()

        # Construct the CANCoder from either a tuple of motor ID and CAN bus ID or just a motor ID
        try:
            self._encoder = phoenix6.hardware.CANcoder(*id_)
        except TypeError:
            self._encoder = phoenix6.hardware.CANcoder(id_)

        configs = phoenix6.configs.CANcoderConfiguration()
        configs.magnet_sensor.absolute_sensor_discontinuity_point = 1  # Set sensor range between 0 and 360 degrees
        configs.magnet_sensor.sensor_direction = phoenix6.signals.SensorDirectionValue(invert)

        # Configs are automatically factory-defaulted
        self._encoder.configurator.apply(configs)

        self._position_signal = self._encoder.get_absolute_position()

        wpilib.SmartDashboard.putData(f"Absolute CANCoder {id_}", self)

    @property
    def absolute_position(self) -> Rotation2d:
        # Convert rotations to degrees
        return Rotation2d.fromDegrees(self._position_signal.refresh().value * 360)


class AbsoluteDutyCycleEncoder(AbsoluteEncoder):
    def __init__(self, dio_pin: int):
        super().__init__()

        self._encoder = wpilib.DutyCycleEncoder(dio_pin)
        wpilib.SmartDashboard.putData(f"Absolute PWM Encoder {dio_pin}", self)

    @property
    def absolute_position_degrees(self) -> float:
        pos = self._encoder.get()  # 0.0 <= pos < 1.0 (rotations)
        degrees = 360 * pos
        return degrees

    @property
    def absolute_position(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.absolute_position_degrees)

    def reset_zero_position(self):
        self._encoder.setPositionOffset(0)


class PigeonGyro(Gyro):
    def __init__(self, id_: int, invert: bool = False):
        super().__init__()

        self._gyro = phoenix5.sensors.PigeonIMU(id_)
        self.invert = invert

        self._sim_gyro = self._gyro.getSimCollection()

        wpilib.SmartDashboard.putData("Pigeon IMU", self)

    def zero_heading(self):
        self._gyro.setYaw(0)

    def simulation_periodic(self, delta_position: float):
        self._sim_gyro.addHeading(delta_position * 180 / math.pi)

    @property
    def heading(self) -> Rotation2d:
        yaw = self._gyro.getYaw()
        if self.invert:
            yaw = 360 - yaw
        return Rotation2d.fromDegrees(yaw)


class Pigeon2Gyro(Gyro):
    def __init__(self, id_: int | tuple[int, str], invert: bool = False):
        super().__init__()
        self.invert = invert

        try:
            # Unpack tuple of sensor id and CAN bus id into Pigeon2 constructor
            self._gyro = phoenix6.hardware.Pigeon2(*id_)
        except TypeError:
            # Only an int was provided for id_
            self._gyro = phoenix6.hardware.Pigeon2(id_)

        self._gyro_sim = self._gyro.sim_state
        self._yaw_signal = self._gyro.get_yaw()

        wpilib.SmartDashboard.putData("Pigeon 2", self)

    def zero_heading(self):
        self._gyro.reset()

    def simulation_periodic(self, delta_position: float):
        # Convert delta_position from radians to degrees and add it into the simulation
        self._gyro_sim.add_yaw(delta_position * 180 / math.pi)

    @property
    def heading(self) -> Rotation2d:
        yaw = self._yaw_signal.refresh().value
        if self.invert:
            yaw = 360 - yaw
        return Rotation2d.fromDegrees(yaw)


class DummyGyro(Gyro):
    """Gyro that does nothing on a real robot but functions normally in simulation"""

    def __init__(self, *args):
        super().__init__()
        self._radians = 0

    def zero_heading(self):
        self._radians = 0

    def simulation_periodic(self, delta_position: float):
        self._radians += delta_position

    @property
    def heading(self) -> Rotation2d:
        return Rotation2d(self._radians)


class SparkMaxEncoderType(enum.Enum):
    ANALOG = enum.auto()
    PWM = enum.auto()


class SparkMaxAbsoluteEncoder(AbsoluteEncoder):
    def __init__(self, controller: rev.SparkMax, encoder_type: SparkMaxEncoderType):
        """
        Absolute encoder plugged into the SPARK MAX's data port

        :param controller: SPARK MAX instance
        :param encoder_type: Type of encoder plugged in. Based on how the encoder transmits data
        """

        super().__init__()

        # Two types of absolute encoders can be plugged into the SPARK MAX data port: analog and duty cycle/PWM
        if encoder_type is SparkMaxEncoderType.ANALOG:
            self._encoder = controller.getAnalog()
        elif encoder_type is SparkMaxEncoderType.PWM:
            self._encoder = controller.getAbsoluteEncoder()

        settings = rev.SparkBaseConfig()
        # Analog encoders output from 0V - 3.3V
        # Change from voltage to degrees
        settings.analogSensor.positionConversionFactor(360 / 3.3)
        # Duty cycle encoders output from 0 to 1 by default
        # Change into degrees
        settings.absoluteEncoder.positionConversionFactor(360)
        controller.configure(
            settings,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

        wpilib.SmartDashboard.putData(f"Absolute Encoder {controller.getDeviceId()}", self)

    @property
    def absolute_position(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self._encoder.getPosition())

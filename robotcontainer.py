import math
import logging

logger = logging.getLogger("your.robot")

from rev import SparkMax
import wpilib
from wpilib import PS4Controller
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
import commands2

from swervepy import u, SwerveDrive, TrajectoryFollowerParameters
from swervepy.impl import CoaxialSwerveModule

from constants import PHYS, MECH, ELEC, OP, SW, DS
import components
from commands import elevatormove, miscdriver, climberupdown
from commands2 import InstantCommand, RunCommand


class RobotContainer:
    """
    This example robot container should serve as a demonstration for how to
    implement swervepy on your robot.  You should not need to edit much of the
    code in this module to get a test working.  Instead, edit the values and
    class choices in constants.py.
    """

    def __init__(self):
        from subsystems.limelight_camera import LimelightCamera
        from subsystems.climber import Climber
        from subsystems.elevator import Elevator
        
        self.gyro = components.gyro_component_class(**components.gyro_param_values)

        #initialize subsystems
        self.camera = LimelightCamera("limelight-pickup")  # TODO: name of your camera goes in parentheses

        #self.climber = Climber(ELEC.Climber_CAN_ID)
        self.elevator = Elevator(leadMotorCANId=ELEC.Elevator_Lead_CAN_ID, presetSwitchPositions=(15, 20, 25), motorClass=SparkMax)

        self.configure_button_bindings()

        # The Azimuth component included the absolute encoder because it needs
        # to be able to reset to absolute position.
        self.lf_enc = components.absolute_encoder_class(ELEC.LF_encoder_DIO, MECH.steering_encoder_inverted)
        self.lb_enc = components.absolute_encoder_class(ELEC.LB_encoder_DIO, MECH.steering_encoder_inverted)
        self.rb_enc = components.absolute_encoder_class(ELEC.RB_encoder_DIO, MECH.steering_encoder_inverted)
        self.rf_enc = components.absolute_encoder_class(ELEC.RF_encoder_DIO, MECH.steering_encoder_inverted)
        modules = (
            # Left Front module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.LF_drive_CAN_ID,
                    parameters=components.drive_params,
                ),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.LF_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(296),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.lf_enc,
                ),
                placement=Translation2d(*components.module_locations["LF"]),
            ),
            # Right Front module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.RF_drive_CAN_ID,
                    parameters=components.drive_params,
                ),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.RF_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(171),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.rf_enc,
                ),
                placement=Translation2d(*components.module_locations["RF"]),
            ),
            # Left Back module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.LB_drive_CAN_ID,
                    parameters=components.drive_params,
                ),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.LB_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(229),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.lb_enc,
                ),
                placement=Translation2d(*components.module_locations["LB"]),
            ),
            # Right Back module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.RB_drive_CAN_ID,
                    parameters=components.drive_params,
                ),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.RB_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(252),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.rb_enc,
                ),
                placement=Translation2d(*components.module_locations["RB"]),
            ),
        )

        self.stick = wpilib.Joystick(0)

        self.speed_limit_ratio = 1.0
        if OP.speed_limit:
            if OP.speed_limit > OP.max_speed:
                wpilib.reportWarning("Speed limit is greater than max_speed and won't be used")
            else:
                self.speed_limit_ratio = OP.speed_limit / OP.max_speed

        self.angular_velocity_limit_ratio = 1.0
        if OP.angular_velocity_limit:
            if OP.angular_velocity_limit > OP.max_angular_velocity:
                wpilib.reportWarning("Angular velocity limit is greater than max_angular_velocity and won't be used")
            else:
                self.angular_velocity_limit_ratio = (
                    OP.angular_velocity_limit / OP.max_angular_velocity)

        # Define a swerve drive subsystem by passing in a list of SwerveModules
        # and some options
        #
        self.swerve = SwerveDrive(modules, self.gyro, OP.max_speed, OP.max_angular_velocity)

        # Set the swerve subsystem's default command to teleoperate using
        # the controller joysticks
        #
        self.swerve.setDefaultCommand(
            self.swerve.teleop_command(
                translation=self.get_translation_input,
                strafe=self.get_strafe_input,
                rotation=self.get_rotation_input,
                field_relative=SW.field_relative,
                drive_open_loop=SW.drive_open_loop,
            )
        )

    def log_data(self):
        for pos in ("LF", "RF", "LB", "RB"):
            encoder = getattr(self, f"{pos.lower()}_enc")
            wpilib.SmartDashboard.putNumber(f"{pos} absolute encoder", encoder.absolute_position_degrees)
            wpilib.SmartDashboard.putNumber(f"{pos} absolute encoder", encoder.absolute_position_degrees)

    @staticmethod
    def deadband(value, band):
        return value if abs(value) > band else 0

    def process_joystick_input(self, val, deadband=0.1, exponent=1, limit_ratio=1.0, invert=False):
        """
        Given a raw joystick reading, return the processed value after adjusting
        for real-world UX considerations:
          * apply a deadband to ignore jitter around zero
          * apply an exponent for greater low-velocity control
        """
        deadbanded_input = self.deadband(val, deadband)
        input_sign = +1 if val > 0 else -1  # this works for val=0 also
        invert_sign = -1 if invert else +1
        # abs required for fractional exponents
        scaled_input = limit_ratio * abs(deadbanded_input) ** exponent
        return invert_sign * input_sign * scaled_input

    def get_translation_input(self, invert=True):
        raw_stick_val = self.stick.getRawAxis(OP.translation_joystick_axis)
        return self.process_joystick_input(raw_stick_val, invert=invert,
                                           limit_ratio=self.speed_limit_ratio)

    def get_strafe_input(self, invert=True):
        raw_stick_val = self.stick.getRawAxis(OP.strafe_joystick_axis)
        return self.process_joystick_input(raw_stick_val, invert=invert,
                                           limit_ratio=self.speed_limit_ratio)

    def get_rotation_input(self, invert=True):
        raw_stick_val = self.stick.getRawAxis(OP.rotation_joystick_axis)
        return self.process_joystick_input(
            raw_stick_val, invert=invert, limit_ratio=self.angular_velocity_limit_ratio)

    def get_autonomous_command(self):
        follower_params = TrajectoryFollowerParameters(
            max_drive_velocity=4.5 * (u.m / u.s),
            theta_kP=1,
            xy_kP=1,
        )

        bezier_points = PathPlannerPath.waypointsFromPoses(
            [
                Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
                Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)),
            ]
        )
        path = PathPlannerPath(
            bezier_points,
            PathConstraints(3.0, 3.0, 2 * math.pi, 4 * math.pi),
            GoalEndState(0.0, Rotation2d.fromDegrees(-90)),  # Zero velocity and facing 90 degrees clockwise
        )

        first_path = True  # reset robot pose to initial pose in trajectory
        open_loop = True  # don't use built-in motor feedback for velocity
        return self.swerve.follow_trajectory_command(path, follower_params, first_path, open_loop)

    # Configure button bindings here
    #TODO: Test this code and see if it works properly (limelight still needs implementation)
    
    def configure_button_bindings(self):
        def turn_to_object():
            x = self.camera.getX()
            print(f"x={x}")
            turn_speed = -0.005 * x
            self.swerve.rotate(turn_speed)
            # if you want your robot to slowly chase that object... replace this line above with: self.robotDrive.arcadeDrive(0.1, turn_speed)

            bButton = self.driverController.button(PS4Controller.Button.kSquare)
            bButton.whileTrue(commands2.RunCommand(turn_to_object, self.swerve.drive))
            bButton.onFalse(commands2.InstantCommand(lambda: self.swerve.drive(0, 0, False, False)))

        # Initialize a seperate controller, on the same port, so I can more easily use bindings, and for code verbosity
        self.driverController = commands2.button.CommandGenericHID(DS.kDriverControllerPort)
        self.buttonboard = commands2.button.CommandGenericHID(DS.kButtonBoardPort)
        
        # Resets gyro heading
        gyroReset = self.driverController.button(PS4Controller.Button.kTriangle)
        gyroReset.onTrue(miscdriver.ResetGyro(self.gyro))

        # Climber up/down
        climberup = self.driverController.pov(0) #TODO: Change this ID
        climberdown = self.driverController.pov(180) #TODO: Change this ID

        # climberup.whileTrue(climberupdown.ClimberMove(1, self.climber)) #TODO: Implement Climber subsystem and commands
        # climberdown.whileTrue(climberupdown.ClimberMove(-1, self.climber)) #TODO: Implement Climber subsystem and commands

        # right stick of the joystick to move the elevator up and down 
        # self.elevator.setDefaultCommand(
        #     commands2.RunCommand(lambda: self.elevator.drive(
        #         self.driverController.getRawAxis(XboxController.Axis.kRightY)
        #     ), self.elevator)
        # )

        # left bumper and right bumper will move elevator between presetSwitchPositions (see above) 
        leftBumper = self.driverController.button(PS4Controller.Button.kL1)
        leftBumper.onTrue(InstantCommand(self.elevator.switchUp, self.elevator))
        rightBumper = self.driverController.button(PS4Controller.Button.kR1)
        rightBumper.onTrue(InstantCommand(self.elevator.switchDown, self.elevator))

        # the "A" button will request elevator to go to a special position of 33.0 inches
        aButton = self.driverController.button(PS4Controller.Button.kCross)
        aButton.onTrue(InstantCommand(lambda: self.elevator.setPositionGoal(33.0), self.elevator))


    
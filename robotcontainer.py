import math, os
import logging

logger = logging.getLogger("your.robot")

from rev import SparkMax
import wpilib
from wpilib import PS4Controller, SmartDashboard
from wpilib.interfaces import GenericHID
from wpimath.geometry import Translation2d, Rotation2d
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import PathPlannerAuto, AutoBuilder, NamedCommands
import commands2

from swervepy import u, SwerveDrive, TrajectoryFollowerParameters
from swervepy.impl import CoaxialSwerveModule

from constants import PHYS, MECH, ELEC, OP, SW, DS, AUTO
import components
from commands import miscdriver, simplecommands, armmove
from commands.drive import resetxy
from commands2 import InstantCommand, RunCommand


class RobotContainer:
    """
    This example robot container should serve as a demonstration for how to
    implement swervepy on your robot.  You should not need to edit much of the
    code in this module to get a test working.  Instead, edit the values and
    class choices in constants.py.
    """

    def __init__(self):
        #other imports, TODO: Will move these
        from subsystems.limelight_camera import LimelightCamera
        from subsystems.climber import Climber
        from subsystems.arm import Arm, ArmConstants
        from subsystems.shooter import Shooter 
        from subsystems.flipper import Flipper
        
        self.gyro = components.gyro_component_class(**components.gyro_param_values)

        #initialize subsystems
        self.camera = LimelightCamera("limelight-pickup")  # TODO: name of your camera goes in parentheses

        self.climber = Climber(ELEC.Climber_CAN_ID)
        self.coralmanip = Arm(ELEC.Arm_Lead_CAN_ID, None) #Arm(ELEC.Arm_Lead_CAN_ID)
        self.shooter = Shooter(ELEC.Shooter_Lead_CAN_ID, ELEC.Shooter_Follow_CAN_ID)
        self.flipper = Flipper(ELEC.Flipper_Lead_CAN_ID)

        # to access in configure_button_bindings
        self.armconsts = ArmConstants

        # The Azimuth component included the absolute encoder because it needs
        # to be able to reset to absolute position.
        
        self.lf_enc = components.absolute_encoder_class(ELEC.LF_encoder_CAN_ID, MECH.steering_encoder_inverted)
        self.lb_enc = components.absolute_encoder_class(ELEC.LB_encoder_CAN_ID, MECH.steering_encoder_inverted)
        self.rb_enc = components.absolute_encoder_class(ELEC.RB_encoder_CAN_ID, MECH.steering_encoder_inverted)
        self.rf_enc = components.absolute_encoder_class(ELEC.RF_encoder_CAN_ID, MECH.steering_encoder_inverted)
        modules = (
            # Left Front module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.LF_drive_CAN_ID,
                    parameters=components.drive_params,
                ),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.LF_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(MECH.LF_Encoder_Offset),
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
                    azimuth_offset=Rotation2d.fromDegrees(MECH.RF_Encoder_Offset),
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
                    azimuth_offset=Rotation2d.fromDegrees(MECH.LB_Encoder_Offset),
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
                    azimuth_offset=Rotation2d.fromDegrees(MECH.RB_Encoder_Offset),
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
        self.swerve = SwerveDrive(modules, self.gyro, OP.max_speed, OP.max_angular_velocity, path_following_params=AUTO)

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
        #self.autoChooser = wpilib.SendableChooser()
        #self.autoChooser.setDefaultOption("Auto 3pts")

        """Add named commands here"""
        """
        #TODO: Check this implementation
        pathsPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
        for file in os.listdir(pathsPath):
            relevantName = file.split(".")[0] # Gets rid of .path/.auto in dropdown
            auton = PathPlannerAuto(relevantName) #makes it an auto
            wpilib.SmartDashboard.putData(f"autos/{relevantName}", auton) #puts in in smartdashboard under autos
            self.autoChooser.addOption(relevantName, auton)

        SmartDashboard.putData("Auto Chooser", self.autoChooser) #puts the chooser in shuffleboard
        """
        self.configure_button_bindings() #check below, adds all commands
        

        

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

    def get_strafe_input(self, invert=False):
        raw_stick_val = self.stick.getRawAxis(OP.strafe_joystick_axis)
        return self.process_joystick_input(raw_stick_val, invert=invert,
                                           limit_ratio=self.speed_limit_ratio)

    def get_rotation_input(self, invert=True):
        raw_stick_val = self.stick.getRawAxis(OP.rotation_joystick_axis)
        return self.process_joystick_input(
            raw_stick_val, invert=invert, limit_ratio=self.angular_velocity_limit_ratio)
    
    def get_auto_command(self):
        #path = PathPlannerPath.fromPathFile(self.autoChooser.getSelected())
        path = PathPlannerPath.fromPathFile("Auto 3pts") #just selects the most basic auto, for testing purposes
        return AutoBuilder.followPath(path)

    #TODO: Test Limelight code
    def makeAlignWithAprilTagCommand(self):
        from commands.drive.setcamerapipeline import SetCameraPipeline
        from commands.drive.followobject import FollowObject, StopWhen
        from commands.drive.approach import ApproachTag
        from commands.drive.swervetopoint import SwerveToSide

        # switch to camera pipeline 3, to start looking for certain kind of AprilTags
        lookForTheseTags = SetCameraPipeline(self.camera, 3)
        approachTheTag = FollowObject(self.camera, self.swerve, stopWhen=StopWhen(maxSize=4), speed=0.3)  # stop when tag size=4 (4% of the frame pixels)
        alignAndPush = ApproachTag(self.camera, self.swerve, None, speed=0.5, pushForwardSeconds=1.0)  # tuning this at speed=0.5, should be comfortable setting speed=1.0 instead

        # or you can do this, if you want to score the coral 15 centimeters to the right and two centimeters back from the AprilTag
        stepToSide = SwerveToSide(drivetrain=self.swerve, metersToTheLeft=-0.15, metersBackwards=0.02, speed=0.2)
        alignToScore = lookForTheseTags.andThen(approachTheTag).andThen(alignAndPush).andThen(stepToSide)

        return alignToScore

    def configure_button_bindings(self):
        """
        Configures and checks for button presses. Below are all the functions that each 
        button does on a PS4 Controller, and the button board. Update this regularly.

        Analog or Digital Sticks
        PS4 Right Stick (not here, in swervepy): Stationary drive rotation
        PS4 Left Stick (not here, in swervepy): Swerve Drive 
        Button Board Stick Vertical: Move elevator up and down at a constant rate
        Button Board Stick Horizontal: not implemented

        Bumpers and triggers (PS4)
        Left Bumper (L1): Moves to preset switch position (should be top) TODO: check this
        Right Bumper (R1): Moves to preset switch position (should be bottom) TODO: check this
        Left Analog Trigger (L2): not implemented
        Right Analog Trigger (R2): not implemented

        Buttons (PS4)
        Cross: Turn arm to set zero point
        Square: turn to apriltag
        Circle: Turn arm to set scoring point (45)
        Triangle: Reset gyro heading

        povUp: Climber up (moves the robot down)
        povDown: Climber down (moves the robot up)
        povLeft: not implemented
        povRight: not implemented
        (no diagonals are used)

        Buttons (Button Board)
        ID1: not implemented
        ID2: not implemented
        ID3: Move elevator to 33 inches TODO: change this to buttonboard for each height of coral
        ID4:
        ID5:
        ID6:
        ID7:
        ID8:
        ID9:
        ID10:
        """
        """
        def turn_to_object():
            x = self.camera.getX()
            print(f"x={x}")
            turn_speed = -0.005 * x
            self.swerve.rotate(turn_speed)
            # if you want your robot to slowly chase that object... replace this line above with: self.robotDrive.arcadeDrive(0.1, turn_speed)

            bButton = self.driverController.button(PS4Controller.Button.kSquare)
            bButton.whileTrue(commands2.RunCommand(turn_to_object, self.swerve.drive))
            bButton.onFalse(commands2.InstantCommand(lambda: self.swerve.drive(0, 0, False, False)))
        """
        # Initialize a seperate controller, on the same port, so I can more easily use bindings, and for code verbosity
        self.driverController = commands2.button.CommandGenericHID(DS.kDriverControllerPort)
        self.buttonboard = commands2.button.CommandGenericHID(DS.kButtonBoardPort)
        
        # Resets gyro heading
        gyroReset = self.driverController.button(PS4Controller.Button.kTriangle)
        gyroReset.onTrue(miscdriver.ResetGyro(self.gyro))

        # Climber up/down
        climberup = self.driverController.pov(180)
        climberdown = self.driverController.pov(0)
        climberup.whileTrue(simplecommands.ClimberMove(1, self.climber))
        climberdown.whileTrue(simplecommands.ClimberMove(-1, self.climber))
        
        manualAxisDown = self.buttonboard.axisLessThan(1, -.9) #axes on buttonboard are inverted, we installed it upside down
        manualAxisDown.whileTrue(simplecommands.ArmMove(.2, self.coralmanip)) #moves algae manip up

        manualAxisUp = self.buttonboard.axisGreaterThan(1, .9)
        manualAxisUp.whileTrue(simplecommands.ArmMove(-0.2, self.coralmanip)) #moves algae manip down
        """
        autoAxisDown = self.buttonboard.button(3) 
        autoAxisDown.onTrue(armmove.ArmMove(self.coralmanip, self.armconsts.kArmMinAngle)) #automatically moves to min angle

        autoAxisUp = self.buttonboard.button(8) #TODO: Check
        autoAxisUp.onTrue(armmove.ArmMove(self.coralmanip, self.armconsts.kArmMaxAngle)) #automatically moves to max angle
        """
        #manual shooting
        shootButton = self.buttonboard.button(1)
        feedbutton = self.buttonboard.button(2)
        shootButton.whileTrue(simplecommands.Shoot(1, self.shooter))
        feedbutton.whileTrue(simplecommands.Shoot(-0.5, self.shooter))

        #Coral Flipper
        flipUp = self.buttonboard.button(4)
        flipDown = self.buttonboard.button(5)
        flipUp.onTrue(simplecommands.FlipCoral(self.flipper, .25))
        flipDown.onTrue(simplecommands.FlipCoral(self.flipper, -.25))

#https://docs.google.com/document/d/1Uh3FElSqB26P4WG2fAvQatyAOpxFWfXhSTuvogocAvs/edit?tab=t.0#bookmark=id.cxhyhp336yos

import logging, math

logger = logging.getLogger("6932Log")

import wpilib
from wpilib import PS4Controller, SmartDashboard
from wpilib.interfaces import GenericHID
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import PathPlannerAuto, AutoBuilder, NamedCommands
import commands2

from swervepy import u, SwerveDrive

from constants import ELEC, OP, SW, DS, AUTO
import components
from commands import miscdriver, simplecommands, armmove
from commands.drive import resetxy
from commands2 import InstantCommand, RunCommand

from utils import makeSwerveComponents


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
        from subsystems.arm2 import Arm, ArmConstants
        from subsystems.shooter import Shooter 
        from subsystems.flipper import Flipper
        
        self.gyro = components.gyro_component_class(**components.gyro_param_values)

        #initialize subsystems
        self.camera = LimelightCamera("limelight-pickup")  # TODO: name of your camera goes in parentheses

        self.climber = Climber(ELEC.Climber_CAN_ID)
        self.coralmanip = Arm(ELEC.Arm_Lead_CAN_ID) #Arm(ELEC.Arm_Lead_CAN_ID, None) 
        self.shooter = Shooter(ELEC.Shooter_Lead_CAN_ID, ELEC.Shooter_Follow_CAN_ID)
        self.flipper = Flipper(ELEC.Flipper_Lead_CAN_ID)

        # to access in configure_button_bindings
        self.armconsts = ArmConstants

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

        # Define a swerve drive subsystem by passing in a list of SwerveModules and some options
        self.swerve = SwerveDrive(makeSwerveComponents(), self.gyro, OP.max_speed, OP.max_angular_velocity, path_following_params=AUTO)

        # Set the swerve subsystem's default command to teleoperate using the controller joysticks
        self.swerve.setDefaultCommand(
            self.swerve.teleop_command(
                translation=self.get_translation_input,
                strafe=self.get_strafe_input,
                rotation=self.get_rotation_input,
                field_relative=SW.field_relative,
                drive_open_loop=SW.drive_open_loop,
            )
        )
        self.autoChooser = AutoBuilder.buildAutoChooser("Auto 3pts")

        """Add auto named commands here"""

        self.configure_button_bindings()
        #TODO: implement Debug mode, test if this code works
        if SW.debug_Mode == True:
            ...

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
        path = PathPlannerPath.fromPathFile(self.autoChooser.getSelected())
        # path = PathPlannerPath.fromPathFile("The dumbest path ever") #just selects the most basic auto, for testing purposes
        return AutoBuilder.followPath(path)
    

    def configure_button_bindings(self):
        """
        Configures and checks for button presses. Below are all the functions that each 
        button does on a PS4 Controller, and the button board. Update this regularly.

        Analog or Digital Sticks
        PS4 Right Stick (not here, in swervepy): Stationary drive rotation
        PS4 Left Stick (not here, in swervepy): Swerve Drive 
        Button Board Stick Vertical: Move arm vertically (speed control)
        Button Board Stick Horizontal: not implemented

        Bumpers and triggers (PS4)
        Left Bumper (L1): not implemented
        Right Bumper (R1): not implemented
        Left Analog Trigger (L2): not implemented
        Right Analog Trigger (R2): not implemented

        Buttons (PS4)
        Cross: not implemented
        Square: not implemented
        Circle: not implemented
        Triangle: Reset gyro heading

        povUp: Climber up (moves the robot down)
        povDown: Climber down (moves the robot up)
        povLeft: not implemented
        povRight: not implemented
        (no diagonals are used)

        Buttons (Button Board)
        ID1: Feed arm intake
        ID2: Shoot arm Intake
        ID3: 
        ID4: Flip Coral Flipper Up
        ID5: Flip Coral Flipper Down
        ID6:
        ID7:
        ID8:
        ID9:
        ID10:
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
        
        #Move the arm up and down, manually
        manualAxisDown = self.buttonboard.axisLessThan(1, -.9) #axes on buttonboard are inverted, we installed it upside down
        manualAxisDown.whileTrue(armmove.ArmMove(self.coralmanip, -.3, True)) #moves algae manip up

        manualAxisUp = self.buttonboard.axisGreaterThan(1, .9)
        manualAxisUp.whileTrue(armmove.ArmMove(self.coralmanip, .3, True)) #moves algae manip down
        """
        #move the arm to setpoints, automatically
        autoAxisDown = self.buttonboard.button(6) 
        autoAxisDown.onTrue(armmove.ArmMove(self.coralmanip, self.armconsts.kArmMinAngle)) #automatically moves to min angle

        autoAxisUp = self.buttonboard.button(10)
        autoAxisUp.onTrue(armmove.ArmMove(self.coralmanip, self.armconsts.kArmMaxAngle)) #automatically moves to max angle
        """
        #shooting
        shootButton = self.buttonboard.button(1)
        feedbutton = self.buttonboard.button(2)
        shootButton.whileTrue(simplecommands.Shoot(1, self.shooter))
        feedbutton.whileTrue(simplecommands.Shoot(-0.5, self.shooter))

        #Coral Flipper
        flipUp = self.buttonboard.button(5)
        flipDown = self.buttonboard.button(4)
        flipUp.whileTrue(simplecommands.FlipCoral(self.flipper, .25))
        flipDown.whileTrue(simplecommands.FlipCoral(self.flipper, -.25))

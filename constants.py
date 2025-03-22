"""
This file defines constants related to your robot.  These constants include:

 * Physical constants (exterior dimensions, wheelbase)

 * Mechanical constants (gear reduction ratios)

 * Electrical constants (current limits, CAN bus IDs, roboRIO slot numbers)

 * Operation constants (desired max velocity, max turning speed)

 * Software constants (USB ID for driver joystick)
"""

import math
from collections import namedtuple
import rev, phoenix5
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
from pathplannerlib.config import RobotConfig

from swervepy import u


config = RobotConfig.fromGUISettings()

# Physical constants
phys_data = {
    "track_width": (21.73 * u.inch).m_as(u.m),
    "wheel_base": (21.73 * u.inch).m_as(u.m),
    "wheel_circumference": 4 * math.pi * u.inch,
}
PHYS = namedtuple("Data", phys_data.keys())(**phys_data)


# Mechanical constants
mech_data = {
    "swerve_module_propulsion_gearing_ratio": 6.12,  # SDS Mk4i L3
    "swerve_module_steering_gearing_ratio": 150 / 7,  # SDS Mk4i

    "propulsion_motor_inverted": True,
    "steering_motor_inverted": False,
    "steering_encoder_inverted": False,

    "RF_Encoder_Offset": 320, #19 
    "RB_Encoder_Offset": 335, #16 
    "LB_Encoder_Offset": 254, #18 
    "LF_Encoder_Offset": 338, #17 

    #should be in phys, but meh
    "kDriveKinematics": SwerveDrive4Kinematics(Translation2d(phys_data["wheel_base"] / 2, phys_data["track_width"] / 2), 
                                               Translation2d(phys_data["wheel_base"] / 2, -phys_data["track_width"] / 2), 
                                               Translation2d(-phys_data["wheel_base"] / 2, phys_data["track_width"] / 2), 
                                               Translation2d(-phys_data["wheel_base"] / 2, -phys_data["track_width"] / 2))
    
}
MECH = namedtuple("Data", mech_data.keys())(**mech_data)

# Electrical constants
elec_data = {
    # These current limit parameters are per-motor in the swerve modules
    "drive_continuous_current_limit": 40,
    "azimuth_continuous_current_limit": 30,
    "vortex_continuous_current_limit": 20,
    "drive_peak_current_limit": 60,
    "azimuth_peak_current_limit": 40,
    "vortex_peak_current_limit": 30,

    # Talon FX motor controllers can set peak_current_duration.
    # SparkMAX motor controllers can't.
    "drive_peak_current_duration": 0.01,
    "azimuth_peak_current_duration": 0.01,

    # time in seconds for propulsion motors to ramp up to full speed
    # reference: https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax
    "open_loop_ramp_rate": 0.5,
    "closed_loop_ramp_rate": 0.5,

    #this is assuming use of a can-based encoder solution, such as cancoders on mk4is
    #because of some serious tomfoolery with the coordinate system in this lib, left and right need to be switched in
    #code compared to how it is physically. If that still doesn't resolve it, try combinations based on stationary rotation.
    "RF_steer_CAN_ID":   9,
    "RF_drive_CAN_ID":   8,
    "RF_encoder_CAN_ID": 19,
    "RB_steer_CAN_ID":   2,
    "RB_drive_CAN_ID":   1,
    "RB_encoder_CAN_ID": 18,
    "LB_steer_CAN_ID":   4,
    "LB_drive_CAN_ID":   3,
    "LB_encoder_CAN_ID": 16,
    "LF_steer_CAN_ID":   6,
    "LF_drive_CAN_ID":   5,
    "LF_encoder_CAN_ID": 17,

    "Climber_CAN_ID": 12,
    "Arm_Lead_CAN_ID": 7, 
    "Shooter_Lead_CAN_ID": 14, 
    "Shooter_Follow_CAN_ID": 13,
    "Flipper_Lead_CAN_ID": 35, #really high, just didn't feel like figuring out the lowest lmao

    "Gyro_CAN_ID": 20,
    "Invert_Gyro": True,
}
ELEC = namedtuple("Data", elec_data.keys())(**elec_data)

JOYSTICK_AXES = {
    "LEFT_X": 0,
    "LEFT_Y": 1,
    "RIGHT_X": 2,
    "RIGHT_Y": 5,
}

# Operation constants
op_data = {
    # These maximum parameters reflect the maximum physically possible, not the
    # desired maximum limit.
    "max_speed": 5.0 * (u.m / u.s),
    "max_angular_velocity": 13.5 * (u.rad / u.s),

    # You can limit how fast your robot moves (e.g. during testing) using the
    # following parameters.  Setting to None is the same as setting to
    # max_speed/max_angular_velocity, and indicates no limit.
    
    "speed_limit": 1.75 * (u.m / u.s), #the highest controllable value that our drivers were comfortable with this year.
    "angular_velocity_limit": 2.5 * (u.rad / u.s),

    # For NEO / SparkMAX, use the following
    "propulsion_neutral": rev.SparkMax.IdleMode.kBrake,
    "steering_neutral": rev.SparkMax.IdleMode.kBrake,

    # Values to pass to stick.getRawAxis()
    # Set these according to your operator preferences
    "translation_joystick_axis": JOYSTICK_AXES["LEFT_Y"],
    "strafe_joystick_axis": JOYSTICK_AXES["LEFT_X"],
    "rotation_joystick_axis": JOYSTICK_AXES["RIGHT_X"],
}
OP = namedtuple("Data", op_data.keys())(**op_data)

# Software constants
sw_data = {
    # field_relative: True if "forward" means "down the field"; False if
    # "forward" means "in the direction the robot is facing".  A True value
    # requires a (non-Dummy) gyro.
    "field_relative": True,

    # drive_open_loop: True if we're not using PID control *for velocity targeting*,
    # i.e. when a target velocity is calculated, do we use the corresponding
    # CoaxialDriveComponent's follow_velocity_open() method (set motor output
    # proportionally based on target and max velocities) or
    # follow_velocity_closed() method (put motor in PID control mode and set
    # target velocity).
    #
    "drive_open_loop": True,

    # Constants for PID control of the propulsion AND steering motors
    # (kP must be non-zero, or azimuth motors won't engage.)
    "kP": 0.012,   # representative value for NEO motors
    "kI": 0,
    "kD": 0.001,

    # Constants for feed-forward of propulsion motors
    "kS": 0,
    "kV": 0,
    "kA": 0,
}
SW = namedtuple("Data", sw_data.keys())(**sw_data)

DS_data = {
    "kDriverControllerPort": 0,
    "kButtonBoardPort": 1,
}
DS = namedtuple("Data", DS_data.keys())(**DS_data)

#You shouldn't have to change this.
AUTO_data = {
    "xy_kP": 0.012,
    "theta_kP": 1,
    "drive_open_loop": True,
}
AUTO = namedtuple("Data", AUTO_data.keys())(**AUTO_data)
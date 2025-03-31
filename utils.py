"""
This file defines usefule functions for the entire robot project. Included functions are:
 * getLeadMotorConfig: creates a SparkMaxConfig class based on params
 * makeSwerveComponents: create a swerve drivebase (only ever called once)
"""

#https://docs.google.com/document/d/1Uh3FElSqB26P4WG2fAvQatyAOpxFWfXhSTuvogocAvs/edit?tab=t.0#bookmark=id.hlvhy6ikm6z9

from rev import SparkMaxConfig, SparkBaseConfig, ClosedLoopConfig

import components
from swervepy.impl import CoaxialSwerveModule
from wpimath.geometry import Translation2d, Rotation2d
from constants import ELEC, MECH
from collections import namedtuple

"""Just a file to define various useful functions for the entire project. This includes motor configuration to save lines,
swerve definition instead of it being in robotcontainer, and other various utilities."""

def getLeadMotorConfig(
                invert: bool,
                idleMode: SparkBaseConfig.IdleMode,
                continuousCurrentLimit: float,
                peakCurrentLimit: float,
                relPositionConversionFactor: float,
                relVelocityConversionFactor: float,
                PID: list,
                feedBackSensor: ClosedLoopConfig.FeedbackSensor,
                reverseSoftLimit: float,
                forwardSoftLimit: float,) -> SparkMaxConfig:
    """Takes relevant parameters for a lead REV motor, then returns a SparkMaxConfig Class. This can then be used to 
    configure a motor Mostly written because configuring a motor is a lot of boilerplate code. (also, only relative encoders)
    :param invert: Takes a bool and inverts the motor if true
    :param idleMode: Takes a SparkBaseConfig.IdleMode (either kCoast or kBrake)
    :param continuousCurrentLimit: Takes a float to determine the max continuous current.
    :param peakCurrentLimit: Takes a float to determine the max peak current.
    :param relPositionConversionFactor: Takes a float value to determine the position conversion factor for the relative encoder.
    :param relVelocityConversionFactor: Takes a float value to determine the velocity conversion factor for the relative encoder.
    :param PID: Optional, Takes a list (in order PIDF) and applies PID values. (NOTE: Feedforward is optional)
    :param feedBackSensor: Optional, determines the feedback sensor used in PID. Necessary if PID is used.
    :param reverseSoftLimit: Optional, takes a float and creates a soft limit when motor is running in reverse.
    :param forwardSoftLimit: Optional, takes a float and creates a soft limit when motor is running in forward.
    
    :returns: A SparkMaxConfig, configured based on the set parameters."""

    config = SparkMaxConfig()
    config.inverted(invert)
    config.setIdleMode(idleMode)
    config.smartCurrentLimit(continuousCurrentLimit)
    config.secondaryCurrentLimit(peakCurrentLimit)
    config.encoder.positionConversionFactor(relPositionConversionFactor)
    config.encoder.velocityConversionFactor(relVelocityConversionFactor)  # 60 seconds per minute

    if reverseSoftLimit and forwardSoftLimit:
        config.softLimit.reverseSoftLimit(reverseSoftLimit)
        config.softLimit.forwardSoftLimit(forwardSoftLimit)
    config.closedLoop.setFeedbackSensor(feedBackSensor)

    if PID:
        config.closedLoop.pid(PID[0], PID[1], PID[2])
        if PID[3] != None:
            config.closedLoop.velocityFF(PID[3])
        config.closedLoop.outputRange(-1.0, 1.0)
        if feedBackSensor:
            config.closedLoop.setFeedbackSensor(feedBackSensor)

    return config

def makeSwerveComponents() -> tuple:
    """Creates the components used for swervepy's implementation of swervedrives. Returns the swerve modules
    as a tuple of items that are directly used as parameters"""
    # gyro = components.gyro_component_class(**components.gyro_param_values)

    lf_enc = components.absolute_encoder_class(ELEC.LF_encoder_CAN_ID, MECH.steering_encoder_inverted)
    lb_enc = components.absolute_encoder_class(ELEC.LB_encoder_CAN_ID, MECH.steering_encoder_inverted)
    rb_enc = components.absolute_encoder_class(ELEC.RB_encoder_CAN_ID, MECH.steering_encoder_inverted)
    rf_enc = components.absolute_encoder_class(ELEC.RF_encoder_CAN_ID, MECH.steering_encoder_inverted)
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
                    absolute_encoder=lf_enc,
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
                    absolute_encoder=rf_enc,
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
                    absolute_encoder=lb_enc,
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
                    absolute_encoder=rb_enc,
                ),
                placement=Translation2d(*components.module_locations["RB"]),
            ),
        )

    return modules

def debug() -> None:
    ...
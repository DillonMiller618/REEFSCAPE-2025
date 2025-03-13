import commands2

from commands.drive.aimtodirection import AimToDirection, AimToDirectionConstants
from commands.drive.gotopoint import GoToPointConstants

from swervepy.subsystem import SwerveDrive
from wpimath.geometry import Rotation2d
from wpilib import Timer


class SetCameraPipeline(commands2.Command):

    def __init__(self, camera, pipelineIndex=0, onlyTagIds=()):
        super().__init__()
        self.pipelineIndex = pipelineIndex
        self.onlyTagIds = onlyTagIds
        self.camera = camera
        self.addRequirements(camera)

    def initialize(self):
        # if camera allows to set filter to look for specific tag IDs, filter for them
        if hasattr(self.camera, "setOnlyTagIds"):
            self.camera.setOnlyTagIds(self.onlyTagIds)
        # if camera has "setPipeline", set it
        if hasattr(self.camera, "setPipeline"):
            self.camera.setPipeline(self.pipelineIndex)

    def isFinished(self) -> bool:
        # if camera has no "setPipeline", we have nothing to wait for
        if not hasattr(self.camera, "setPipeline"):
            return True
        # we are finished when the camera has responded that pipeline index is now set
        if self.camera.getPipeline() == self.pipelineIndex:
            return True
        # otherwise, print that we aren't finished
        print("SetCameraPipeline: not yet finished, because camera pipeline = {} and we want {}".format(
            self.camera.getPipeline(), self.pipelineIndex)
        )
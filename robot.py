#https://docs.google.com/document/d/1Uh3FElSqB26P4WG2fAvQatyAOpxFWfXhSTuvogocAvs/edit?tab=t.0#bookmark=id.7sxe087kbu7y

from typing import Optional
import commands2
from robotcontainer import RobotContainer


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.autonomous_command: Optional[commands2.Command] = None

    def autonomousInit(self) -> None:
        self.autonomous_command = self.container.get_auto_command()
        if self.autonomous_command:
            self.autonomous_command.schedule()    

    def teleopInit(self) -> None:
        if self.autonomous_command:
            self.autonomous_command.cancel()

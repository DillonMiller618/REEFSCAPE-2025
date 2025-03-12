import commands2 


#This file is ONLY for driver commands that don't fit into the position commands that are required for scoring.

class ResetGyro(commands2.Command):
    def __init__(self, gyro):
        """
        Reset the gyro position to (angle)
        :gyro angle: Angle to set
        """
        super().__init__()
        self.gyro = gyro

    def initialize(self):
        self.gyro.zero_heading()
    
    def isFinished(self) -> bool:
        return True
    
    def execute(self):
        pass
        #does it in initialize
    
    def end(self, interruped: bool):
        pass
        #instant command, nothing here



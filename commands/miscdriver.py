import commands2 


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
        ...
        #does it in initialize
    
    def end(self, interruped: bool):
        ...
        #instant command, nothing here



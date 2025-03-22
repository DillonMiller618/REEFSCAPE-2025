from rev import SparkMax, SparkBase, SparkBaseConfig, ClosedLoopConfig, SparkMaxConfig, ClosedLoopSlot
from wpimath.geometry import Rotation2d
from commands2 import Subsystem
from wpilib import SmartDashboard

class ArmConstants:
    # very scary setting! if set wrong, the arm will escape equilibrium and break something
    kEncoderInverted = False

    # one full revolution = 360 units (since we want degree units)
    kEncoderPositionFactor = 360

    # we want speed in degrees per second, not RPM
    kEncoderPositionToVelocityFactor = 1.0 / 60

    # calculating how many motor revolutions are needed to move arm by one degree
    gearReduction = 80.0
    fudgeFactor = 1  # empirical, if needed
    motorRevolutionsPerDegree = gearReduction / 360 * fudgeFactor

    kArmMinAngle = 0 #zero point
    kArmMaxAngle = 105 #is climber angle
    kArmScoringAngle = 60 #should be a 45
    kArmMaxWeightAngle = 30 #idk
    kAngleTolerance = 2.0 #idk

    # PID coefficients
    initialStaticGainTimesP = 3.5  # we are normally this many degrees off because of static forces
    initialP = .3
    initialI = 0.0
    initialD = 0.0
    additionalPMult = 3.0  # when close to target angle

    initialMaxOutput = 1
    initialMinOutput = -1
    initialMaxRPM = 5700

    # Smart Motion Coefficients, but maybe they apply outside of SmartMotion too?
    initialMaxVel = 2000  # rpm
    initialMinVel = -2000  # rpm
    initialMaxAcc = 2500
    initialAllowedError = .02  # was 0.02

    # Hacks
    kAngleGoalRadius = 10
    kExtraDelayForOscillationsToStop = 0.1  # seconds (until the PID coefficients below are tuned to avoid oscillations)


class Arm(Subsystem):
    def __init__(self, leadMotorCANId: int) -> None:
        super().__init__()
        self.leadmotor = SparkMax(leadMotorCANId, SparkBase.MotorType.kBrushless)
        self.pidcontroller = self.leadmotor.getClosedLoopController()
        self.encoder = self.leadmotor.getEncoder()

        self.leadmotor.configure(self._getLeadMotorConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        self.encoder.setPosition(0)
        self.angleGoal = ArmConstants.kArmMaxAngle #starts at high position, e.g. vertical
        self.pidcontroller.setReference(self.angleGoal, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0)


    def periodic(self) -> None:
        SmartDashboard.putNumber("Current Angle", self.angleGoal)

    def getPosition(self) -> float:
        """Gets the angle of the arm from the relative encoder in rotations"""
        return self.encoder.getPosition()
    
    def checkAngle(self, tolerance) -> bool:
        """Checks if the current angle is close to the setpoint, with a tolerance factor"""
        pass
    
    def setAngleGoal(self, angle: Rotation2d):
        self.angleGoal = angle.degrees()
        #keeping the angle in bounds by setting the angle if it is past normal operating range to the edge of operating.
        if self.angleGoal < ArmConstants.kArmMinAngle:
            self.angleGoal = ArmConstants.kArmMinAngle
        elif self.angleGoal > ArmConstants.kArmMaxAngle:
            self.angleGoal = ArmConstants.kArmMaxAngle

        self.pidcontroller.setReference(self.angleGoal, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0)

    def setSpeed(self, speed: float) -> None:
        """Legacy method of arm, just in case PID control isn't responding properly"""
        self.leadmotor.set(speed)
    
    def stopMotor(self) -> None:
        """Used to completely stop the motor, including speed and pid controls"""
        self.leadmotor.stopMotor()


    def _getLeadMotorConfig(invert=False) -> SparkMaxConfig:

        config = SparkMaxConfig()
        config.inverted(False)
        config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        relPositionFactor = 1.0 / ArmConstants.motorRevolutionsPerDegree
        config.encoder.positionConversionFactor(relPositionFactor)
        config.encoder.velocityConversionFactor(relPositionFactor / 60)  # 60 seconds per minute

        config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)

        assert ArmConstants.kArmMaxAngle > ArmConstants.kArmMinAngle, (
            f"ArmConstants.kArmMaxAngle={ArmConstants.kArmMaxAngle} is not greater than " +
            f"ArmConstants.kArmMinAngle={ArmConstants.kArmMinAngle}"
        )
        config.softLimit.reverseSoftLimit(ArmConstants.kArmMinAngle)
        config.softLimit.forwardSoftLimit(ArmConstants.kArmMaxAngle)

        config.closedLoop.pid(ArmConstants.initialP, ArmConstants.initialI, ArmConstants.initialD)
        config.closedLoop.velocityFF(0.0)  # because position control
        config.closedLoop.outputRange(ArmConstants.initialMinOutput, ArmConstants.initialMaxOutput)

        return config
            
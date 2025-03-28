from rev import SparkMax, SparkBase, SparkBaseConfig, LimitSwitchConfig, ClosedLoopConfig
from wpimath.geometry import Rotation2d
from commands2 import Subsystem
from wpilib import SmartDashboard

# constants right here, to simplify the video
class ArmConstants:
    # very scary setting! if set wrong, the arm will escape equilibrium and break something
    kEncoderInverted = False

    # one full revolution = 360 units (since we want degree units)
    kEncoderPositionFactor = 360

    # we want speed in degrees per second, not RPM
    kEncoderPositionToVelocityFactor = 1.0 / 60

    # calculating how many motor revolutions are needed to move arm by one degree
    chainSprocket = 1  # teeth
    driveSprocket = 1  # teeth
    gearReduction = 80.0
    chainReduction = chainSprocket / driveSprocket
    fudgeFactor = 1  # empirical, if needed
    motorRevolutionsPerDegree = gearReduction * chainReduction / 360 * fudgeFactor

    kArmMinAngle = 0 #zero point
    kArmMaxAngle = 105 #is climber angle
    kArmScoringAngle = 60 #should be a 45
    kArmMaxWeightAngle = 30 #idk
    kAngleTolerance = 2.0 #idk

    # PID coefficients
    initialStaticGainTimesP = 3.5  # we are normally this many degrees off because of static forces
    initialD = 0.0
    initialP = .3
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
    def __init__(
        self,
        leadMotorCANId: int,
        followMotorCANId: int,
        dontSlam: bool=False,
        limitSwitchType=LimitSwitchConfig.Type.kNormallyOpen,  # make NormallyOpen if you don't have limit switches yet
        dashboardPrefix="",  # set it to "front_" if this is a front arm in a multi-arm robot
    ) -> None:
        """Constructs an arm. Be very very careful with setting PIDs -- arms are dangerous"""
        super().__init__()
        self.dontSlam = dontSlam
        self.armAngleLabel = dashboardPrefix + "armAngle"
        self.armAngleGoalLabel = dashboardPrefix + "armAngleGoal"
        self.armPositionLabel = dashboardPrefix + "armPosition"
        self.armStateLabel = dashboardPrefix + "armState"

        self.leadMotor = SparkMax(leadMotorCANId, SparkMax.MotorType.kBrushless)
        self.leadMotor.configure(
            _getLeadMotorConfig(limitSwitchType, ArmConstants.kEncoderPositionFactor, ArmConstants.kEncoderInverted),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        self.forwardLimit = self.leadMotor.getForwardLimitSwitch()
        self.reverseLimit = self.leadMotor.getReverseLimitSwitch()

        self.followMotor = None
        if followMotorCANId is not None:
            self.followMotor = SparkMax(followMotorCANId, SparkMax.MotorType.kBrushless)
            followConfig = SparkBaseConfig()
            followConfig.follow(leadMotorCANId, invert=True)
            self.followMotor.configure(
                followConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters)

        # now initialize pid controller and encoder
        self.pidController = self.leadMotor.getClosedLoopController()
        self.encoder = self.leadMotor.getEncoder() #This should be absolute, we don't have any
        # self.relativeEncoder = self.leadMotor.getEncoder()

        # first angle goal
        self.angleGoal = ArmConstants.kArmMinAngle
        self.setAngleGoal(self.angleGoal)


    def periodic(self) -> None:
        SmartDashboard.putNumber(self.armAngleGoalLabel, self.angleGoal)
        SmartDashboard.putNumber(self.armAngleLabel, self.getAngle())
        SmartDashboard.putNumber("MinAngle", ArmConstants.kArmMinAngle)
        SmartDashboard.putString(self.armStateLabel, self.getState())


    def getState(self) -> str:
        if self.forwardLimit.get():
            return "forward limit" if not self.reverseLimit.get() else "both limits (CAN disconn?)"
        if self.reverseLimit.get():
            return "reverse limit"
        # otherwise, everything is ok
        return "ok"


    def isDoneMoving(self) -> bool:
        return abs(self.getAngleGoal() - self.getAngle()) < ArmConstants.kAngleTolerance


    def getAngleGoal(self) -> float:
        return self.angleGoal


    def getAngle(self) -> float:
        return self.encoder.getPosition()


    def getAngleVelocity(self) -> float:
        return self.encoder.getVelocity()


    def setAngleGoal(self, angle: float) -> None:
        self.angleGoal = angle
        """
        if self.angleGoal < ArmConstants.kArmMinAngle:
            self.angleGoal = ArmConstants.kArmMinAngle
        if self.angleGoal > ArmConstants.kArmMaxAngle:
            self.angleGoal = ArmConstants.kArmMaxAngle
        
        if self.dontSlam and self.angleGoal <= ArmConstants.kArmMinAngle:
            
            # we don't want to slam the arm on the floor, but the target angle is pretty low
            print("1")
            if self.getAngle() > ArmConstants.kArmMinAngle:
                print("2")
                self.pidController.setReference(ArmConstants.kArmMinAngle, SparkBase.ControlType.kPosition)
            else:
                print("2.5")
                self.stopAndReset()
        else:
            print("3", self.dontSlam, self.angleGoal)
            # static forces for the arm depend on arm angle (e.g. if it's at the top, no static forces)
            adjustment = ArmConstants.initialStaticGainTimesP * \
                         Rotation2d.fromDegrees(self.angleGoal - ArmConstants.kArmMinAngle).cos()
            self.pidController.setReference(self.angleGoal + adjustment, SparkBase.ControlType.kPosition)
        """

    def startArm(self, speed):
        self.leadMotor.set(speed)

    def stopAndReset(self) -> None:
        self.leadMotor.stopMotor()
        if self.followMotor is not None:
            self.followMotor.stopMotor()
        self.leadMotor.clearFaults()
        if self.followMotor is not None:
            self.followMotor.clearFaults()


def _getLeadMotorConfig(
    limitSwitchType: LimitSwitchConfig.Type,
    absPositionFactor: float,
    absEncoderInverted: bool,
) -> SparkBaseConfig:

    config = SparkBaseConfig()
    config.inverted(False)
    config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.limitSwitch.reverseLimitSwitchEnabled(False)
    config.limitSwitch.forwardLimitSwitchType(limitSwitchType)
    config.limitSwitch.reverseLimitSwitchType(limitSwitchType)

    relPositionFactor = 1.0 / ArmConstants.motorRevolutionsPerDegree
    config.encoder.positionConversionFactor(relPositionFactor)
    config.encoder.velocityConversionFactor(relPositionFactor / 60)  # 60 seconds per minute

    config.absoluteEncoder.positionConversionFactor(absPositionFactor)
    config.absoluteEncoder.velocityConversionFactor(absPositionFactor / 60)  # 60 seconds per minute
    config.absoluteEncoder.inverted(absEncoderInverted)
    config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)

    assert ArmConstants.kArmMaxAngle > ArmConstants.kArmMinAngle, (
        f"ArmConstants.kArmMaxAngle={ArmConstants.kArmMaxAngle} is not greater than " +
        f"ArmConstants.kArmMinAngle={ArmConstants.kArmMinAngle}"
    )
    config.softLimit.reverseSoftLimit(ArmConstants.kArmMinAngle)
    config.softLimit.forwardSoftLimit(ArmConstants.kArmMaxAngle)

    config.closedLoop.pid(ArmConstants.initialP, 0.0, ArmConstants.initialD)
    config.closedLoop.velocityFF(0.0)  # because position control
    config.closedLoop.outputRange(ArmConstants.initialMinOutput, ArmConstants.initialMaxOutput)

    return config
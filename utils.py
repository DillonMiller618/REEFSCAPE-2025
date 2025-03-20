from rev import SparkMaxConfig, SparkBaseConfig, ClosedLoopConfig

def _getLeadMotorConfig(
                relPositionConversionFactor: float,
                motorRevolutionsPerDegree: float,
                reverseSoftLimit: float,
                forwardSoftLimit: float,
                PID: list) -> SparkMaxConfig:

        config = SparkMaxConfig()
        config.inverted(False)
        config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        relPositionFactor = 1.0 / motorRevolutionsPerDegree
        config.encoder.positionConversionFactor(relPositionConversionFactor)
        config.encoder.velocityConversionFactor(relPositionConversionFactor / 60)  # 60 seconds per minute

        config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        if reverseSoftLimit and forwardSoftLimit:
            config.softLimit.reverseSoftLimit(reverseSoftLimit)
            config.softLimit.forwardSoftLimit(forwardSoftLimit)

        config.closedLoop.pid(PID[1], PID[2], PID[3])
        config.closedLoop.velocityFF(0.0)  # because position control
        config.closedLoop.outputRange(-1.0, 1.0)

        return config
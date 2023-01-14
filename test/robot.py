import rev, wpilib


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.kCanID = 1
        self.kMotorType = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.kAltEncType = rev.SparkMaxAlternateEncoder.Type.kQuadrature
        self.kCPR = 8192

        self.m_pidController = rev.SparkMaxPIDController()

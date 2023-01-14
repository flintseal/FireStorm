import rev, ctre, wpilib, robotpy_apriltag, ntcore


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # TODO: The Future of Autonomous: https://robotpy.readthedocs.io/projects/pathplannerlib/en/stable/api.html
        # Network Table Configuration
        self.networkTablesServer = ntcore.NetworkTableInstance.getDefault()
        self.networkTablesServer.startServer()
        self.robotDataNetworkTable = self.networkTablesServer.getTable("RobotData")

        # Variable Mode Initialisation
        self.netTestMode = self.networkTablesServer.getStringTopic("TestMode").subscribe("PFC")
        self.netMotorOutputMode = self.networkTablesServer.getStringTopic("MotorOutputMode").subscribe("%")

        # Initialise Motor Output Data
        self.netLeftMotor = self.networkTablesServer.getDoubleTopic("LeftMotorOutput").publish()
        self.netRightMotor = self.networkTablesServer.getDoubleTopic("RightMotorOutput").publish()
        self.netBackLeftMotor = self.networkTablesServer.getDoubleTopic("BackLeftMotorOutput").publish()
        self.netBackRightMotor = self.networkTablesServer.getDoubleTopic("BackRightMotorOutput").publish()

        # Initialise Controller Output Data
        self.networkTableLeftY = self.robotDataNetworkTable.getDoubleTopic("Left-Y").publish()
        self.networkTableRightY = self.robotDataNetworkTable.getDoubleTopic("Right-Y").publish()

        # Motor Configuration
        # TODO: Ask if follow mode is preferable
        self.LeftMotor = ctre.TalonSRX(1)
        self.RightMotor = ctre.TalonSRX(2)
        self.backLeftMotor = ctre.TalonSRX(3)
        self.backRightMotor = ctre.TalonSRX(4)
        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        # TODO: Test Follow System
        self.backLeftMotor.follow(self.LeftMotor)
        self.backRightMotor.follow(self.RightMotor)

        # Controller Configuration
        self.pilotsStick = wpilib.Joystick(1)
        self.xboxController = wpilib.XboxController(0)

    def robotPeriodic(self):
        # Publish Important Controller Values
        self.networkTableLeftY.set(self.xboxController.getLeftY())
        self.networkTableRightY.set(self.xboxController.getRightY())

        # Publish Motor Output
        if self.netMotorOutputMode == "%":
            self.netLeftMotor.set(self.LeftMotor.getMotorOutputPercent())
            self.netRightMotor.set(self.RightMotor.getMotorOutputPercent())
            self.netBackLeftMotor.set(self.backLeftMotor.getMotorOutputPercent())
            self.netBackRightMotor.set(self.backRightMotor.getMotorOutputPercent())
        elif self.netMotorOutputMode == "V":
            self.netLeftMotor.set(self.LeftMotor.getMotorOutputVoltage())
            self.netRightMotor.set(self.RightMotor.getMotorOutputVoltage())
            self.netBackLeftMotor.set(self.backLeftMotor.getMotorOutputVoltage())
            self.netBackRightMotor.set(self.backRightMotor.getMotorOutputVoltage())
        elif self.netMotorOutputMode == "C":
            self.netLeftMotor.set(self.LeftMotor.getOutputCurrent())
            self.netRightMotor.set(self.RightMotor.getOutputCurrent())
            self.netBackLeftMotor.set(self.backLeftMotor.getOutputCurrent())
            self.netBackRightMotor.set(self.backRightMotor.getOutputCurrent())
        elif self.netMotorOutputMode == "W":
            self.netLeftMotor.set(self.LeftMotor.getOutputCurrent()*self.LeftMotor.getMotorOutputVoltage())
            self.netRightMotor.set(self.RightMotor.getOutputCurrent()*self.RightMotor.getMotorOutputVoltage())
            self.netBackLeftMotor.set(self.backLeftMotor.getOutputCurrent()*self.backLeftMotor.getMotorOutputVoltage())
            self.netBackRightMotor.set(self.backRightMotor.getOutputCurrent()*self.backRightMotor.getMotorOutputVoltage())

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        self.xboxController.getLeftY()
        self.xboxController.getRightY()

    def disabledInit(self):
        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)

    def disabledPeriodic(self):
        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def testInit(self):
        if self.netTestMode.get() == "PFC":
            pass
        else:
            pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)

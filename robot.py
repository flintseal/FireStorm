import rev, ctre, wpilib, robotpy_apriltag, ntcore


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # TODO: The Future of Autonomous: https://robotpy.readthedocs.io/projects/pathplannerlib/en/stable/api.html
        # Network Table Configuration
        self.networkTablesServer = ntcore.NetworkTableInstance.getDefault()
        self.networkTablesServer.startServer()
        self.robotDataNetworkTable = self.networkTablesServer.getTable("RobotData")

        # Initialise Percent Output Data
        self.netLeftMotorPercentage = self.networkTablesServer.getDoubleTopic("LeftMotorOutput").publish()
        self.netRightMotorPercentage = self.networkTablesServer.getDoubleTopic("RightMotorOutput").publish()
        self.netBackLeftMotorPercentage = self.networkTablesServer.getDoubleTopic("BackLeftMotorOutput").publish()
        self.netBackRightMotorPercentage = self.networkTablesServer.getDoubleTopic("BackRightMotorOutput").publish()

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
        # self.LeftMotor.getMotorOutputPercent()
        self.networkTableLeftY.set(self.xboxController.getLeftY())
        self.networkTableRightY.set(self.xboxController.getRightY())

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
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)

import rev, ctre, wpilib, robotpy_apriltag, ntcore


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Hardware Configuration
        self.powerDistribution = wpilib.PowerDistribution(module=0, moduleType=wpilib.PowerDistribution.ModuleType(1))

        # Network Table Configuration
        self.networkTablesServer = ntcore.NetworkTableInstance.getDefault()
        self.networkTablesServer.startServer()
        self.robotDataNetworkTable = self.networkTablesServer.getTable("RobotData")
        self.robotPowerReadoutNetworkTable = self.networkTablesServer.getTable("PowerReadouts")

        # Variable Mode and Readout Initialisation
        self.netPDTemperature = self.robotPowerReadoutNetworkTable.getStringTopic("PD Temperature").publish()
        self.netPDCurrent = self.robotPowerReadoutNetworkTable.getStringTopic("PD Total Amps").publish()
        self.netPDEnergy = self.robotPowerReadoutNetworkTable.getStringTopic("PD Total Joules").publish()
        self.netPDPower = self.robotPowerReadoutNetworkTable.getStringTopic("PD Total Watts").publish()
        self.netPDVoltage = self.robotPowerReadoutNetworkTable.getStringTopic("PD Input Voltage").publish()
        self.netMFRTemperature = self.robotPowerReadoutNetworkTable.getStringTopic("MFR Temperature").publish()
        self.netMFLTemperature = self.robotPowerReadoutNetworkTable.getStringTopic("MFL Temperature").publish()
        self.netMBRTemperature = self.robotPowerReadoutNetworkTable.getStringTopic("MBR Temperature").publish()
        self.netMBLTemperature = self.robotPowerReadoutNetworkTable.getStringTopic("MBL Temperature").publish()
        self.netMFRBusVoltage = self.robotPowerReadoutNetworkTable.getStringTopic("MFR Bus Voltage").publish()
        self.netMFLBusVoltage = self.robotPowerReadoutNetworkTable.getStringTopic("MFL Bus Voltage").publish()
        self.netMBRBusVoltage = self.robotPowerReadoutNetworkTable.getStringTopic("MBR Bus Voltage").publish()
        self.netMBLBusVoltage = self.robotPowerReadoutNetworkTable.getStringTopic("MBL Bus Voltage").publish()
        self.netTestReadout = self.robotDataNetworkTable.getStringTopic("TestReadout").publish()
        self.netTestMode = self.robotDataNetworkTable.getStringTopic("TestMode").subscribe("PFC")
        self.netMotorOutputMode = self.robotDataNetworkTable.getStringTopic("MotorOutputMode").subscribe("%")

        # Initialise Motor Output Data
        self.netLeftMotor = self.robotDataNetworkTable.getDoubleTopic("LeftMotorOutput").publish()
        self.netRightMotor = self.robotDataNetworkTable.getDoubleTopic("RightMotorOutput").publish()
        self.netBackLeftMotor = self.robotDataNetworkTable.getDoubleTopic("BackLeftMotorOutput").publish()
        self.netBackRightMotor = self.robotDataNetworkTable.getDoubleTopic("BackRightMotorOutput").publish()

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

        # Controller Configuration
        self.pilotsStick = wpilib.Joystick(1)
        self.xboxController = wpilib.XboxController(0)

        # Create Timestamp & Other Variables
        self.testTimestamp = 0
        self.pfcTestStages = ["Init", "Motors", "Sensors", "Electrical"]
        self.pfcTestStage = None
        self.pfcTestStageComplete = False
        self.stickyBrownoutTriggered = False
        if wpilib.DriverStation.isFMSAttached():
            self.stickyBrownoutEnabled = False
        else:
            self.stickyBrownoutEnabled = True

    def robotPeriodic(self):
        if wpilib.RobotController.isBrownedOut():
            wpilib.reportError("Brownout Alert")
            if self.stickyBrownoutEnabled:
                self.stickyBrownoutTriggered = True
        # Publish Important Controller Values
        self.networkTableLeftY.set(self.xboxController.getLeftY())
        self.networkTableRightY.set(self.xboxController.getRightY())

        # Publish Motor Output
        # TODO: Test this with real robot
        if self.netMotorOutputMode == "V":
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
            self.netLeftMotor.set(self.LeftMotor.getOutputCurrent() * self.LeftMotor.getMotorOutputVoltage())
            self.netRightMotor.set(self.RightMotor.getOutputCurrent() * self.RightMotor.getMotorOutputVoltage())
            self.netBackLeftMotor.set(
                self.backLeftMotor.getOutputCurrent() * self.backLeftMotor.getMotorOutputVoltage())
            self.netBackRightMotor.set(
                self.backRightMotor.getOutputCurrent() * self.backRightMotor.getMotorOutputVoltage())
        else:
            self.netLeftMotor.set(self.LeftMotor.getMotorOutputPercent())
            self.netRightMotor.set(self.RightMotor.getMotorOutputPercent())
            self.netBackLeftMotor.set(self.backLeftMotor.getMotorOutputPercent())
            self.netBackRightMotor.set(self.backRightMotor.getMotorOutputPercent())

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        if self.stickyBrownoutTriggered:
            self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
            self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
            self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
            self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        else:
            self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=self.xboxController.getLeftY())
            self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=self.xboxController.getLeftY())
            self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=self.xboxController.getRightY())
            self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=self.xboxController.getRightY())

    def disabledInit(self):
        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)

        # Set Unneeded Readouts to Disabled
        self.netTestReadout.set("Disabled")
        self.netPDTemperature.set("Disabled")
        self.netPDCurrent.set("Disabled")
        self.netPDEnergy.set("Disabled")
        self.netPDPower.set("Disabled")
        self.netPDVoltage.set("Disabled")
        self.netMFRTemperature.set("Disabled")
        self.netMFLTemperature.set("Disabled")
        self.netMBRTemperature.set("Disabled")
        self.netMBLTemperature.set("Disabled")
        self.netMFRBusVoltage.set("Disabled")
        self.netMFLBusVoltage.set("Disabled")
        self.netMBRBusVoltage.set("Disabled")
        self.netMBLBusVoltage.set("Disabled")

    def disabledPeriodic(self):
        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)

    def autonomousInit(self):
        # TODO: The Future of Autonomous: https://robotpy.readthedocs.io/projects/pathplannerlib/en/stable/api.html
        pass

    def autonomousPeriodic(self):
        if self.stickyBrownoutTriggered:
            self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
            self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
            self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
            self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        else:
            pass

    def testInit(self):
        if self.netTestMode.get() == "MF":
            self.testTimestamp = wpilib.Timer.getFPGATimestamp()
        else:
            self.testTimestamp = None
            self.pfcTestStage = 0

    def testPeriodic(self):
        if self.stickyBrownoutTriggered:
            self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
            self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
            self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
            self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        else:
            if self.netTestMode.get() == "MF":
                # TODO: Create Move Forward Protocol
                pass
            else:
                if self.pfcTestStage == 0:
                    self.netTestReadout.set("PFC: Press A to Continue to stage 1")
                if self.xboxController.getAButton() is True and self.pfcTestStage == 0:
                    self.testTimestamp = int(wpilib.Timer.getFPGATimestamp())
                    self.pfcTestStageComplete = False
                    self.pfcTestStage = 1
                    self.netTestReadout.set("PFC: Testing Motors Please Wait")
                elif self.xboxController.getAButton() is True and self.pfcTestStage == 1 and self.pfcTestStageComplete:
                    self.testTimestamp = int(wpilib.Timer.getFPGATimestamp())
                    self.pfcTestStageComplete = False
                    self.pfcTestStage = 2
                    self.netTestReadout.set("PFC: Power Readouts sent to table")
                if self.xboxController.getAButton() is True and self.pfcTestStage == 0:
                    self.testTimestamp = int(wpilib.Timer.getFPGATimestamp())
                    self.pfcTestStageComplete = False
                    self.pfcTestStage = 1
                    self.netTestReadout.set("PFC: Testing Motors Please Wait")
                if self.pfcTestStage == 1 and not self.pfcTestStageComplete:
                    if 0 < (wpilib.Timer.getFPGATimestamp() - self.testTimestamp) < 2:
                        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0.5)
                        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    elif 2 < (wpilib.Timer.getFPGATimestamp() - self.testTimestamp) < 4:
                        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0.5)
                        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    elif 4 < (wpilib.Timer.getFPGATimestamp() - self.testTimestamp) < 6:
                        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0.5)
                        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    elif 6 < (wpilib.Timer.getFPGATimestamp() - self.testTimestamp) < 8:
                        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0.5)
                    elif 8 < (wpilib.Timer.getFPGATimestamp() - self.testTimestamp) < 10:
                        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=-0.5)
                        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    elif 10 < (wpilib.Timer.getFPGATimestamp() - self.testTimestamp) < 12:
                        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=-0.5)
                        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    elif 12 < (wpilib.Timer.getFPGATimestamp() - self.testTimestamp) < 14:
                        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=-0.5)
                        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    elif 14 < (wpilib.Timer.getFPGATimestamp() - self.testTimestamp) < 16:
                        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=-0.5)
                    else:
                        self.pfcTestStageComplete = True
                elif self.pfcTestStage == 1 and self.pfcTestStageComplete:
                    self.netTestReadout.set("PFC: Press A to Continue to stage 2")
                elif self.pfcTestStage == 2:
                    self.sendElectricalData()
                # TODO: Add Sensor Tests

    def sendElectricalData(self):
        self.netPDTemperature.set(str(self.powerDistribution.getTemperature()))
        self.netPDCurrent.set(str(self.powerDistribution.getTotalCurrent()))
        self.netPDEnergy.set(str(self.powerDistribution.getTotalEnergy()))
        self.netPDPower.set(str(self.powerDistribution.getTotalPower()))
        self.netPDVoltage.set(str(self.powerDistribution.getVoltage()))
        self.netMFRTemperature.set(str(self.LeftMotor.getTemperature()))
        self.netMFLTemperature.set(str(self.backLeftMotor.getTemperature()))
        self.netMBRTemperature.set(str(self.RightMotor.getTemperature()))
        self.netMBLTemperature.set(str(self.backRightMotor.getTemperature()))
        self.netMFRBusVoltage.set(str(self.LeftMotor.getBusVoltage()))
        self.netMFLBusVoltage.set(str(self.backLeftMotor.getBusVoltage()))
        self.netMBRBusVoltage.set(str(self.RightMotor.getBusVoltage()))
        self.netMBLBusVoltage.set(str(self.backRightMotor.getBusVoltage()))


if __name__ == "__main__":
    wpilib.run(MyRobot)

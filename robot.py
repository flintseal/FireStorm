import ctre
import ntcore
import robotpy_apriltag
import wpilib


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # April Tag Config
        self.aprilTagConfig = robotpy_apriltag.AprilTagDetector.Config()
        self.aprilTagQuadConfig = robotpy_apriltag.AprilTagDetector.QuadThresholdParameters()

        # Config General Image Processing
        self.aprilTagConfig.debug = True  # Turn off when performing
        self.aprilTagConfig.decodeSharpening = 0.25  # Sharpening for Small Tags
        self.aprilTagConfig.numThreads = 2  # Number of threads used to detect
        self.aprilTagConfig.quadDecimate = 2  # Turn up for performance but less accuracy and detection rate
        self.aprilTagConfig.quadSigma = 0  # Turn up for Noisy Images by 0.1
        self.aprilTagConfig.refineEdges = True  # Increase Accuracy if quad_decimate != 0

        # Quadrature Threshold Parameters (Fine Tuning of Angles and Detection Rate)
        self.aprilTagQuadConfig.criticalAngle = 10  # Tag Corner Angle that the detector will reject tags
        self.aprilTagQuadConfig.deglitch = False  # Turn on if picture is Very Noisy

        # Create and Define Detector Object's Properties
        self.aprilTagDetector = robotpy_apriltag.AprilTagDetector()  # Create April Tag Detector Object
        self.aprilTagDetector.setConfig(self.aprilTagConfig)  # Config General Settings
        self.aprilTagDetector.setQuadThresholdParameters(self.aprilTagQuadConfig)  # Config Quad Settings
        self.aprilTagDetector.addFamily("tag16h5")  # Define Tag Family in use

        # Pneumatics Config
        self.pneumaticsController = wpilib.PneumaticHub(10)
        self.compressor = self.pneumaticsController.makeCompressor()
        self.armSolenoidLeft = self.pneumaticsController.makeSolenoid(0)  # TODO: Locate bus location
        self.armSolenoidRight = self.pneumaticsController.makeSolenoid(1)  # TODO: Locate bus location

        # Encoder Config
        self.leftSideEncoder = wpilib.Encoder(0, 1)
        self.rightSideEncoder = wpilib.Encoder(2, 3)

        # Camera Server
        wpilib.CameraServer.launch()

        # Hardware Configuration
        self.powerDistribution = wpilib.PowerDistribution(module=0, moduleType=wpilib.PowerDistribution.ModuleType(1))

        # Network Table Configuration
        self.networkTablesServer = ntcore.NetworkTableInstance.getDefault()
        self.networkTablesServer.startServer()
        self.robotDataNetworkTable = self.networkTablesServer.getTable("RobotData")
        self.robotPowerReadoutNetworkTable = self.networkTablesServer.getTable("PowerReadouts")
        self.robotPIDNetworkTable = self.networkTablesServer.getTable("PID Tuning")

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
        self.LeftMotor = ctre.TalonSRX(3)
        self.RightMotor = ctre.TalonSRX(4)
        self.backLeftMotor = ctre.TalonSRX(1)
        self.backRightMotor = ctre.TalonSRX(2)
        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        
        # PID Tuning Variables
        # TODO: Make better default values
        self.netPIDProportionalGain = self.robotPIDNetworkTable.getFloatTopic("Proportional Gain").subscribe(defaultValue=0.5)
        self.netPIDIntegralGain = self.robotPIDNetworkTable.getFloatTopic("Integral Gain").subscribe(defaultValue=0.5)

        # Apriltag Data
        self.netAprilTagLocationY = self.robotDataNetworkTable.getStringTopic("AprilTagLocationX").subscribe("None")
        self.netAprilTagLocationX = self.robotDataNetworkTable.getStringTopic("AprilTagLocationY").subscribe("None")
        self.netAprilTagLocationT = self.robotDataNetworkTable.getStringTopic("AprilTagLocationTimestamp").subscribe("None")

        # Controller Configuration
        # TODO: Test and setup joystick for everything but drive
        self.pilotsStick = wpilib.Joystick(1)
        self.xboxController = wpilib.XboxController(0)

        # Create Timestamp & Other Variables
        self.previousErrorValueLeft = 0
        self.previousErrorValueRight = 0
        self.testTimestamp = 0
        self.pfcTestStages = ["Init", "Motors", "Sensors", "Electrical"]
        self.pfcTestStage = None
        self.pfcTestStageComplete = False
        self.stickyBrownoutTriggered = False
        self.lastATTimestamp = 0
        self.desiredTestRotationsValue = 1000
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
        if self.netMotorOutputMode == "V":
            self.netLeftMotor.set(self.LeftMotor.getMotorOutputVoltage())
            self.netRightMotor.set(self.RightMotor.getMotorOutputVoltage())
            self.netBackLeftMotor.set(self.backLeftMotor.getMotorOutputVoltage())
            self.netBackRightMotor.set(-self.backRightMotor.getMotorOutputVoltage())
        elif self.netMotorOutputMode == "C":
            self.netLeftMotor.set(self.LeftMotor.getOutputCurrent())
            self.netRightMotor.set(self.RightMotor.getOutputCurrent())
            self.netBackLeftMotor.set(self.backLeftMotor.getOutputCurrent())
            self.netBackRightMotor.set(-self.backRightMotor.getOutputCurrent())
        elif self.netMotorOutputMode == "W":
            self.netLeftMotor.set(self.LeftMotor.getOutputCurrent() * self.LeftMotor.getMotorOutputVoltage())
            self.netRightMotor.set(self.RightMotor.getOutputCurrent() * self.RightMotor.getMotorOutputVoltage())
            self.netBackLeftMotor.set(
                self.backLeftMotor.getOutputCurrent() * self.backLeftMotor.getMotorOutputVoltage())
            self.netBackRightMotor.set(
                self.backRightMotor.getOutputCurrent() * -self.backRightMotor.getMotorOutputVoltage())
        else:
            self.netLeftMotor.set(self.LeftMotor.getMotorOutputPercent())
            self.netRightMotor.set(self.RightMotor.getMotorOutputPercent())
            self.netBackLeftMotor.set(self.backLeftMotor.getMotorOutputPercent())
            self.netBackRightMotor.set(-self.backRightMotor.getMotorOutputPercent())

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # TODO: Implement Idiot-Proofing Mode with Equation from Ethan
        if self.stickyBrownoutTriggered:
            self.brownoutDisabler()
        else:
            self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=-self.xboxController.getLeftY())
            self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=-self.xboxController.getLeftY())
            self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=self.xboxController.getRightY())
            self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=self.xboxController.getRightY())

    def disabledInit(self):
        self.leftSideEncoder.reset()
        self.rightSideEncoder.reset()
        self.compressor.disable()
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
        self.compressor.disable()
        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)

    def brownoutDisabler(self):
        self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
        self.compressor.disable()

    def autonomousInit(self):
        # TODO: The Future of Autonomous: https://robotpy.readthedocs.io/projects/pathplannerlib/en/stable/api.html
        pass

    def autonomousPeriodic(self):
        if self.stickyBrownoutTriggered:
            self.brownoutDisabler()
        else:
            yValue = float(self.netAprilTagLocationY.get())
            xValue = float(self.netAprilTagLocationX.get())
            tValue = float(self.netAprilTagLocationT.get())

            if (tValue - self.lastATTimestamp) > 2:
                if xValue > 20:
                    self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0.5)
                # elif xValue < 90:
                #     self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0.5)
                else:
                    self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
            else:
                self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
            self.lastATTimestamp = tValue

    def testInit(self):
        if self.netTestMode.get() == "MF":
            self.testTimestamp = "First"
            self.previousErrorValueLeft = "First"
            self.previousErrorValueRight = "First"
        else:
            self.testTimestamp = None
            self.pfcTestStage = 0

    def testPeriodic(self):
        if self.stickyBrownoutTriggered:
            self.brownoutDisabler()
        else:
            if self.netTestMode.get() == "MF":
                # TODO: Level 3 Waypoints
                # TODO: Test PID
                self.netPIDProportionalGain.get()
                self.netPIDIntegralGain.get()
                if self.testTimestamp == "First":
                    changeInTime = 0
                else:
                    changeInTime = wpilib.Timer.getFPGATimestamp() - self.testTimestamp
                errorValueLeft = (self.desiredTestRotationsValue - self.leftSideEncoder.get())
                errorValueRight = (self.desiredTestRotationsValue - self.rightSideEncoder.get())
                if (self.previousErrorValueLeft == "First") or (self.previousErrorValueRight == "First"):
                    changeInErrorLeft = 0
                    changeInErrorRight = 0
                else:
                    changeInErrorLeft = errorValueLeft - self.previousErrorValueLeft
                    changeInErrorRight = errorValueRight - self.previousErrorValueRight
                controlVariableLeft = (errorValueLeft * self.netPIDProportionalGain.get()) + self.netPIDIntegralGain.get() * (errorValueLeft * changeInTime) + (self.netPIDProportionalGain.get() * (changeInErrorLeft/changeInTime))
                controlVariableRight = (errorValueRight * self.netPIDProportionalGain.get()) + self.netPIDIntegralGain.get() * (errorValueRight * changeInTime) + (self.netPIDProportionalGain.get() * (changeInErrorRight/changeInTime))
                self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=controlVariableLeft)
                self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=controlVariableRight)
                self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=controlVariableLeft)
                self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=controlVariableRight)
                self.testTimestamp = wpilib.Timer.getFPGATimestamp()
                self.previousErrorValueLeft = errorValueLeft
                self.previousErrorValueRight = errorValueRight
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
                    self.LeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    self.backLeftMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    self.RightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    self.backRightMotor.set(mode=ctre.ControlMode.PercentOutput, value=0)
                    self.sendElectricalData()

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

prompt = '''
# Robot Definition Code
#!/usr/bin/env python3

import wpilib
import wpilib.drive
import romi
import math

from robotpy_ext.autonomous import AutonomousModeSelector
class MyRobot(wpilib.TimedRobot):
    """
    This shows using the AutonomousModeSelector to automatically choose
    autonomous modes.

    If you find this useful, you may want to consider using the Magicbot
    framework, as it already has this integrated into it.
    """
    kCountsPerRevolution = 1440.0
    kWheelDiameterInch = 2.75591


    def robotInit(self):
        self.lstick = wpilib.Joystick(0)
        self.rstick = wpilib.Joystick(1)

        # Simple two wheel drive
        self.l_motor = wpilib.Talon(0)
        self.r_motor = wpilib.Talon(1)

        # wheel encoders
        self.l_encoder = wpilib.Encoder(4, 5)
        self.r_encoder = wpilib.Encoder(6, 7)

        # Set up the BuiltInAccelerometer
        self.accelerometer = wpilib.BuiltInAccelerometer()

        self.drive = wpilib.drive.DifferentialDrive(self.l_motor, self.r_motor)

        # Position gets automatically updated as robot moves
        self.gyro = wpilib.AnalogGyro(1)        
        # Set up the RomiGyro
        # self.gyro = romi.RomiGyro()

        # unused
        self.motor = wpilib.Talon(2)
        self.limit1 = wpilib.DigitalInput(1)
        self.limit2 = wpilib.DigitalInput(2)
        self.position = wpilib.AnalogInput(2)

        # Items in this dictionary are available in your autonomous mode
        # as attributes on your autonomous object
        self.components = {"drive": self.drive, "gyro": self.gyro, "motor": self.motor, 
                           "l_motor": self.l_motor, "r_motor": self.r_motor, 
                           "l_encoder": self.l_encoder, "r_encoder": self.r_encoder,
                           "limit1": self.limit1, "limit2": self.limit2, "position": self.position }

        # * The first argument is the name of the package that your autonomous
        #   modes are located in
        # * The second argument is passed to each StatefulAutonomous when they
        #   start up
        self.automodes = AutonomousModeSelector("autonomous", self.components)
        
        # Use inches as unit for encoder distances
        self.l_encoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.r_encoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.resetEncoders()
    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.l_encoder.reset()
        self.r_encoder.reset()

    def getl_encoderCount(self) -> int:
        return self.l_encoder.get()

    def getr_encoderCount(self) -> int:
        return self.r_encoder.get()

    def getLeftDistanceInch(self) -> float:
        return self.l_encoder.getDistance()

    def getRightDistanceInch(self) -> float:
        return self.r_encoder.getDistance()

    def getAverageDistanceInch(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return (self.getLeftDistanceInch() + self.getRightDistanceInch()) / 2.0

    def getAccelX(self) -> float:
        """The acceleration in the X-axis.

        :returns: The acceleration of the Romi along the X-axis in Gs
        """
        return self.accelerometer.getX()

    def getAccelY(self) -> float:
        """The acceleration in the Y-axis.

        :returns: The acceleration of the Romi along the Y-axis in Gs
        """
        return self.accelerometer.getY()

    def getAccelZ(self) -> float:
        """The acceleration in the Z-axis.

        :returns: The acceleration of the Romi along the Z-axis in Gs
        """
        return self.accelerometer.getZ()

    def getGyroAngleX(self) -> float:
        """Current angle of the Romi around the X-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleX()

    def getGyroAngleY(self) -> float:
        """Current angle of the Romi around the Y-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleY()

    def getGyroAngleZ(self) -> float:
        """Current angle of the Romi around the Z-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleZ()

    def resetGyro(self) -> None:
        """Reset the gyro"""
        self.gyro.reset()

    def autonomousInit(self):
        self.drive.setSafetyEnabled(True)
        self.automodes.start()

    def autonomousPeriodic(self):
        self.automodes.periodic()

    def disabledInit(self):
        self.automodes.disable()

    def teleopPeriodic(self):
        """Called when operation control mode is enabled"""

        self.drive.arcadeDrive(self.lstick.getX(), self.lstick.getY())

        # Move a motor with a Joystick
        y = self.rstick.getY()

        # stop movement backwards when 1 is on
        if self.limit1.get():
            y = max(0, y)

        # stop movement forwards when 2 is on
        if self.limit2.get():
            y = min(0, y)

        self.motor.set(y)

# Robot Autonomous Code
from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
class DriveBackwards(StatefulAutonomous):
    MODE_NAME = "Drive Backwards"

    def initialize(self):
        # This allows you to tune the variable via the SmartDashboard over
        # networktables
        self.register_sd_var("drive_speed", -1)

    @timed_state(duration=0.5, next_state="drive_backwards", first=True)
    def drive_wait(self):
        self.drive.tankDrive(0, 0)

    @timed_state(duration=5, next_state="stop")
    def drive_backwards(self):
        self.drive.tankDrive(self.drive_speed, -1 * (self.drive_speed))

    @state()  # Remove or modify this to add additional states to this class.
    def stop(self):
        self.drive.tankDrive(0, 0)

class DriveForward(StatefulAutonomous):
    MODE_NAME = "Drive Forward"

    def initialize(self):
        # This allows you to tune the variable via the SmartDashboard over
        # networktables
        self.register_sd_var("drive_speed", 1)

    @timed_state(duration=0.5, next_state="drive_forward", first=True)
    def drive_wait(self):
        self.drive.tankDrive(0, 0)

    @timed_state(duration=5, next_state="stop")
    def drive_forward(self):
        self.drive.tankDrive(self.drive_speed, -1 * (self.drive_speed))

    @state()  # Remove or modify this to add additional states to this class.
    def stop(self):
        self.drive.tankDrive(0, 0)
'''
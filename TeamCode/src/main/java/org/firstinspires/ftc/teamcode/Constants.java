package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Constants {

    public static class AutoConstants {

        // How far the robot will keep on rolling after braking. Gotten though trial and error and will change based on AUTO_DRIVE_POWER, robot weight, etc.
        public static final double MOVEMENT_BUFFER_INCHES = 1.1;

        // How fast to spin motors while going specified distances in auto. The slower you go the more precise your distance will be.
        public static final double AUTO_DRIVE_POWER = 0.25;

        // Down position for tile dropper, currently a guess
        public static final double DOWN_DROPPER_POSITION = -0.5;

        // up position for tile dropper
        public static final double UP_DROPPER_POSITION = 0.25;

        public static final double RELEASE_AIRPLANE_POSITION = 0.5;


        // arm position close enough
        public static final double ARM_POSITION_RANGE = 5;
    }

    // Constants related to driver preference
    public static class DriverConstants {

        // If grabberSpinIsToggleMode is true then spin state mode will be toggled forward, backward and off, if it is false then you have to hold up/down button.
        public static final boolean grabberSpinIsToggleMode = true;

        // If slowModeIsToggleMode is true then slow mode will be toggled on and off, if it is false then you have to hold slow mode button.
        public static final boolean slowModeIsToggleMode = true;

        // Set to true to turn on ark, false to turn it off.
        public static final boolean enableArk = true;

        // If the power highest power being sent to the wheels is less that minPowerBrakeThreshold, the wheel's ZeroPowerBehavior is set to break
        public static final double minPowerBrakeThreshold = 0.1;

        // If slowModeBrake is true then whenever slow mode is enabled the wheel's ZeroPowerBehavior is set to brake
        public static final boolean slowModeBrake = false;

        // Max power needed to start driving in ark.
        public static final double arkPowerThreshHold = 0.25;

        // ZeroPowerBehavior.BRAKE means that if the wheels are not being sent any power they will try and prevent themselves from moving
        // Default behavior for motors when power is set to 0. FLOAT is means it will keep and rolling and BRAKE means it will try to cancel all external motion
        public static final DcMotor.ZeroPowerBehavior wheelMotorZeroPowerBehaviorDefault = DcMotor.ZeroPowerBehavior.FLOAT;

        // Speed modifiers for different modes. first value is slow mode (B button), middle is normal, last is speed mode (down on respective joystick).
        // Linear speed is for forward, backward, left, right, and diagonal.
        public static final double[] linearSpeedModifiers = {0.25, 0.75, 1};

        // Speed modifiers for different modes. first value is slow mode (B button), middle is normal, last is speed mode (down on respective joystick).
        // Angular speed is for spinning and arks.
        public static final double[] angularSpeedModifiers = {0.1, 0.5, 1};

        // How far trigger needs to be pressed to activate
        public static final double triggerThreshold = 0.9;
    }

    public static class ArmConstants {
        // What power level the amr is set to. Between 0 and 1 with one being max power.
        // This does not control directly speed, it controls how much power the motor will use to get to a desired position
        public static final double armPower = 1;

        // Power of intake motor (spinning device on grabber)
        public static final double intakePower = 0.5;

        // How many encoder ticks the arm moves each time button is pressed.
        // Do not set this to high otherwise the arm may keep on moving even after you stop holding button.
        public static final int armMoveAmount = 3;

        public static final int armPositionShift = 0;

        // Maximum position the arm can go to (Relative to start facing forwards)
        public static final int minPosition = 0 + armPositionShift;

        // Placing set point (Relative to start)
        public static final int maxPosition = 4250 + armPositionShift;

        // Placing set point (Relative to start)
        public static final int armUpSetPoint = 2800 + armPositionShift;

        // Picking up set point (Relative to start)
        public static final int armDownSetPoint = 0 + armPositionShift;

        // In case arm gearing changes
        public static final DcMotor.Direction armDirection = DcMotor.Direction.FORWARD;
    }

    public static class DriveTrainConstants {

        // Set which direction is "forward" for each motor. Use "testMotorsIdDirections" to check these
        public static final DcMotor.Direction leftFrontDriveDirection = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction leftBackDriveDirection = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction rightFrontDriveDirection = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction rightBackDriveDirection = DcMotor.Direction.FORWARD;
    }

    public static class OdometryConstants {

        // Constants about physical robot
        public static final double wheelDiameterMM = 48;
        public static final double ticksInRotation = 2000;
        public static final double wheelCircumferenceMM = wheelDiameterMM * Math.PI;
        public static final double tickInMM = wheelCircumferenceMM / ticksInRotation;

        // Track width and forward offset are for odometer. Currently just place holders
        public static final double TRACK_WIDTH = 177.8 / tickInMM;
        public static final double FORWARD_OFFSET = 69.596 / tickInMM;
    }
}

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Constants {

    public static class ArmConstants {
        // What power level the amr is set to. Between 0 and 1 with one being max power.
        // This does not control directly speed, it controls how much power the motor will use to get to a desired position
        public static  final double armPower = 1;

        // How many encoder ticks the arm moves each time button is pressed
        public static final int armMoveAmount = 3;

        public static final int minPosition = -900;
        public static final int maxPosition = 3800;

        public static final int armUpSetPoint = 2000;
        public static final int armDownSetPoint = -800;

        public static final DcMotor.Direction armDirection = DcMotor.Direction.FORWARD;
    }

    public static class DriveTrainConstants {

        // Set which direction is "forward" for each motor. Use "testMotorsIdDirections" to check these
        public static final DcMotor.Direction leftFrontDriveDirection = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction leftBackDriveDirection = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction rightFrontDriveDirection = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction rightBackDriveDirection = DcMotor.Direction.FORWARD;
    }


    public static class DriverConstants {

        // ZeroPowerBehavior.BRAKE means that if the wheels are not being sent any power they will try and prevent themselves from moving

        // Default behavior for motors when power is set to 0. FLOAT is means it will keep and rolling and BRAKE means it will try to cancel all external motion
        public static final DcMotor.ZeroPowerBehavior wheelMotorZeroPowerBehaviorDefault = DcMotor.ZeroPowerBehavior.FLOAT;

        // If the power highest power being sent to the wheels is less that minPowerBrakeThreshold, the wheel's ZeroPowerBehavior is set to break
        public static final double minPowerBrakeThreshold = 0.1;

        // If slowModeBrake is true then whenever slow mode is enabled the wheel's ZeroPowerBehavior is set to brake
        public static final boolean slowModeBrake = false;

        // Max power needed to start driving in ark.
        public static final double arkPowerThreshHold = 0.25;

        // Set to true to turn on ark, false to turn it off.
        public static final boolean enableArk = true;

        // Speed modifiers for different modes. first value is slow mode, middle is normal, last is speed mode.

        // Linear speed is for forward, backward, left, right, and diagonal.
        public static final double[] linearSpeedModifiers = {0.25, 0.75, 1};

        // Angular speed is for spinning and arks.
        public static final double[] angularSpeedModifiers = {0.1, 0.5, 1};

        // If slowModeIsToggleMode is true then slow mode will be toggled on and off, if it is false then you have to hold slow mode button.
        public static final boolean slowModeIsToggleMode = true;

        // How far trigger needs to be pressed to activate
        public static final double triggerThreshold = 0.9;
    }

    public static class OdometryConstants {
        public static final double wheelDiameterMM = 48;
        public static final double ticksInRotation = 2000;
        public static final double wheelCircumferenceMM = wheelDiameterMM * Math.PI;
        public static final double tickInMM = wheelCircumferenceMM / ticksInRotation;

        public static final double TRACK_WIDTH = 1;
        public static final double FORWARD_OFFSET = 1;
    }
}

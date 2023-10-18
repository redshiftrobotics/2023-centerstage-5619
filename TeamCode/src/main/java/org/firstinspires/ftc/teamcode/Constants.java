package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Constants {

    public static class ArmConstants {
        public static  final double armPower = 0.3;
        public static final int armMoveAmount = 20;
    }

    public static class DriveTrainConstants {
        public static final DcMotor.Direction leftFrontDriveDirection = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction leftBackDriveDirection = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction rightFrontDriveDirection = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction rightBackDriveDirection = DcMotor.Direction.FORWARD;
    }

    public static class DriverConstants {
        public static final DcMotor.ZeroPowerBehavior wheelMotorZeroPowerBehaviorDefault = DcMotor.ZeroPowerBehavior.FLOAT;
        public static final double minPowerBrakeThreshold = 0.1;
        public static final boolean slowModeBrake = true;

        public static final double arkPowerThreshHold = 0.1;

        public static final double[] linearSpeedModifiers = {0.25, 0.75, 1};
        public static final double[] angularSpeedModifiers = {0.1, 0.5, 1};

        public static final boolean shouldStartInSlow = false;
        public static final boolean slowModeIsToggleMode = true;
    }
}

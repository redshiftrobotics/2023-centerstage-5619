package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Odometry Test")
public class RemoteOdometryTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor leftEncoder;
    private DcMotor centerEncoder;
    private DcMotor rightEncoder;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        leftEncoder = hardwareMap.get(DcMotor.class, "LE");
        centerEncoder = hardwareMap.get(DcMotor.class, "CE");
        rightEncoder = hardwareMap.get(DcMotor.class, "RE");

        leftFrontDrive.setDirection(Constants.DriveTrainConstants.leftFrontDriveDirection);
        leftBackDrive.setDirection(Constants.DriveTrainConstants.leftBackDriveDirection);
        rightFrontDrive.setDirection(Constants.DriveTrainConstants.rightFrontDriveDirection);
        rightBackDrive.setDirection(Constants.DriveTrainConstants.rightBackDriveDirection);

        leftFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        leftBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        double prevLeftEncoderPos = 0;
        double prevRightEncoderPos = 0;
        double prevCenterEncoderPos = 0;

        double xPos = 0;
        double yPos = 0;
        double heading = 0;

        while (opModeIsActive()) {
            final double leftEncoderPos = leftEncoder.getCurrentPosition();
            final double rightEncoderPos = rightEncoder.getCurrentPosition();
            final double centerEncoderPos = centerEncoder.getCurrentPosition();

            final double deltaLeftEncoderPos = leftEncoderPos - prevLeftEncoderPos;
            final double deltaRightEncoderPos = rightEncoderPos - prevRightEncoderPos;
            final double deltaCenterEncoderPos = centerEncoderPos - prevCenterEncoderPos;

            final double phi = (deltaLeftEncoderPos - deltaRightEncoderPos) / Constants.OdometryConstants.TRACK_WIDTH;
            final double deltaMiddlePos = (deltaLeftEncoderPos + deltaRightEncoderPos) / 2;
            final double deltaPerpPos = deltaCenterEncoderPos - Constants.OdometryConstants.FORWARD_OFFSET * phi;

            final double deltaX = deltaMiddlePos * Math.cos(heading) - deltaPerpPos * Math.sin(heading);
            final double deltaY = deltaMiddlePos * Math.sin(heading) + deltaPerpPos * Math.cos(heading);

            xPos += deltaX;
            yPos += deltaY;
            heading += phi;

            prevLeftEncoderPos = leftEncoderPos;
            prevRightEncoderPos = rightEncoderPos;
            prevCenterEncoderPos = centerEncoderPos;

            final double linearSpeedModifier = Constants.DriverConstants.linearSpeedModifiers[0];
            final double angularSpeedModifier = Constants.DriverConstants.angularSpeedModifiers[0];


            final double axial = -gamepad1.left_stick_y * linearSpeedModifier;
            final double lateral = gamepad1.left_stick_x * linearSpeedModifier;
            final double yaw = gamepad1.right_stick_x * angularSpeedModifier;

            // Combine joystick requests for each axis-motion to determine each wheel's power.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;


            // Find highest power so we can check if it exceeds the max
            final double maxPower = Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
            );

            // Normalize values so no wheel power never exceeds 100%
            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightBackPower /= maxPower;
            }

            final boolean shouldBrakeStoppedMotors = maxPower < Constants.DriverConstants.minPowerBrakeThreshold;

            if (shouldBrakeStoppedMotors) {
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                leftFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
                leftBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
                rightFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
                rightBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Status", "Run Time: %s", runtime.toString());

            telemetry.addData("Front Power", "Left: %4.2f, Right: %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back Power", "Left: %4.2f, Right: %4.2f", leftBackPower, rightBackPower);

            telemetry.addData("Position", "X: %d, Y: %d, Heading: %d", ticksToInches(xPos), ticksToInches(yPos), ticksToInches(heading));
            telemetry.addData("Delta", "X: %d, Y: %d, Heading: %d", ticksToInches(deltaX), ticksToInches(deltaY), ticksToInches(phi));

            telemetry.addData("Pod Individual", "Left: %d, Center: %d, Right: %d", ticksToInches(leftEncoderPos), ticksToInches(centerEncoderPos), ticksToInches(rightEncoderPos));


            telemetry.update();
        }

    }

    public static int ticksToInches(double mm) {
        return (int) Math.round((mm * Constants.OdometryConstants.tickInMM) / 25.4);
    }
}
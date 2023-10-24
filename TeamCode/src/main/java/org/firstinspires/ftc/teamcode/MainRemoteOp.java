package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Main Remote")
public class MainRemoteOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor arm;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        // Set if motor is reversed
        leftFrontDrive.setDirection(Constants.DriveTrainConstants.leftFrontDriveDirection);
        leftBackDrive.setDirection(Constants.DriveTrainConstants.leftBackDriveDirection);
        rightFrontDrive.setDirection(Constants.DriveTrainConstants.rightFrontDriveDirection);
        rightBackDrive.setDirection(Constants.DriveTrainConstants.rightBackDriveDirection);

        // Enable or disable braking
        leftFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        leftBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);

        arm = hardwareMap.get(DcMotor.class, "top arm");

        // Set arm motor mode
        arm.setDirection(Constants.DriveTrainConstants.armDirection);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(Constants.ArmConstants.armPower);

        boolean isSlowMode = false;
        boolean lastBState = false;

        int targetArmPosition = 0;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(targetArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            // Check if slow mode is enabled and check if it should toggle on or if it must be held
            if (Constants.DriverConstants.slowModeIsToggleMode) {
                if (gamepad1.b && !lastBState) isSlowMode = !isSlowMode;
            }
            else {
                isSlowMode = gamepad1.b;
            }
            lastBState = gamepad1.b;

            // Check which speed modifier mode to be in. Speed modifies just change joystick input
            // Pressing down on stick bumps it up a mode, slow mode bumps it down one, default is middle (1)
            final int linearSpeedMode = 1 + (gamepad1.left_stick_button ? 1 : 0) - (isSlowMode ? 1 : 0);
            final double linearSpeedModifier = Constants.DriverConstants.linearSpeedModifiers[linearSpeedMode];
            final int angularSpeedMode = 1 + (gamepad1.right_stick_button ? 1 : 0) - (isSlowMode ? 1 : 0);
            final double angularSpeedModifier = Constants.DriverConstants.angularSpeedModifiers[angularSpeedMode];

            // Left joystick does linear movement / translation (forward, backward, left, right, diagonals)
            // Right joystick does angular movement / rotation (spinning in place and arks).

            // "axial" is determined by the left stick's y-axis and controls forward and backward motion
            // "lateral" is determined by the left stick's x-axis and controls strafing motion (sideways movement)
            // "yaw" is determined by the right stick's x-axis and controls the rotation of the robot

            final double axial = -gamepad1.left_stick_y * linearSpeedModifier;
            final double lateral = gamepad1.left_stick_x * linearSpeedModifier;
            final double yaw = gamepad1.right_stick_x * angularSpeedModifier;

            // "ark" allows you to spin backward around a point, allowing you to turn around while still moving
            
            double frontArk = gamepad1.right_stick_y * angularSpeedModifier;
            if (!Constants.DriverConstants.enableArk || Math.abs(frontArk) < Constants.DriverConstants.arkPowerThreshHold * angularSpeedModifier)
                frontArk = 0;

            double backArk = 0 * angularSpeedModifier;
            if (!Constants.DriverConstants.enableArk || Math.abs(backArk) < Constants.DriverConstants.arkPowerThreshHold * angularSpeedModifier)
                backArk = 0;

            // Combine joystick requests for each axis-motion to determine each wheel's power.
            double leftFrontPower = axial + lateral + yaw + frontArk;
            double rightFrontPower = axial - lateral - yaw - frontArk;
            double leftBackPower = axial - lateral + yaw + backArk;
            double rightBackPower = axial + lateral - yaw - backArk;


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

            final boolean shouldBrakeStoppedMotors = maxPower < Constants.DriverConstants.minPowerBrakeThreshold
                    || (isSlowMode && Constants.DriverConstants.slowModeBrake);

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

            // Up on dpad to move target position up and down to move target down
            if (gamepad1.dpad_up) targetArmPosition += Constants.ArmConstants.armMoveAmount;
            if (gamepad1.dpad_down) targetArmPosition -= Constants.ArmConstants.armMoveAmount;

            // Send power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Send target position for arm
            arm.setTargetPosition(targetArmPosition);

            // Run time telemetry, mostly just to check if the program is running all right.
            telemetry.addData("Status", "Run Time: %s", runtime.toString());

            // Drive telemetry
            telemetry.addData("Speed Modifiers", "Linear: %.0f%%, Angular: %.0f%%", linearSpeedModifier * 100, angularSpeedModifier * 100);
            telemetry.addData("Front", "Left: %4.2f, Right: %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back ", "Left: %4.2f, Right: %4.2f", leftBackPower, rightBackPower);

            // Device telemetry
            telemetry.addData("Arm Position", "Current: %d, Target: %d", arm.getCurrentPosition(), targetArmPosition);

            telemetry.update();
        }

    }
}
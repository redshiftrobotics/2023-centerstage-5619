package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants.DriverConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.Constants.ArmConstants;

// Wrapper around gamepad for .isPressed() and .isReleased()
// Drive train class to pass around to avoid repeated motor setup

@TeleOp(name="Main Remote")
public class MainRemoteOp extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // Get motors using motor names
    private final DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
    private final DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
    private final DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
    private final DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

    private final DcMotor arm = hardwareMap.get(DcMotor.class, "top arm");

    private int targetArmPosition = 0;

    private boolean isSlowMode = false;
    private boolean lastBState = false;

    public void init() {
        // Set if motor is reversed
        leftFrontDrive.setDirection(DriveTrainConstants.leftFrontDriveDirection);
        leftBackDrive.setDirection(DriveTrainConstants.leftBackDriveDirection);
        rightFrontDrive.setDirection(DriveTrainConstants.rightBackDriveDirection);
        rightBackDrive.setDirection(DriveTrainConstants.rightBackDriveDirection);

        // Enable or disable braking
        leftFrontDrive.setZeroPowerBehavior(DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        leftBackDrive.setZeroPowerBehavior(DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightFrontDrive.setZeroPowerBehavior(DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightBackDrive.setZeroPowerBehavior(DriverConstants.wheelMotorZeroPowerBehaviorDefault);

        // Set arm motor mode
        arm.setDirection(DcMotor.Direction.FORWARD);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(ArmConstants.armPower);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();

        targetArmPosition = 0;
        isSlowMode = DriverConstants.shouldStartInSlow;
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(targetArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
    }

    @Override
    public void loop() {
        // Check if slow mode is enabled and check if it should toggle on or if it must be held
        if (DriverConstants.slowModeIsToggleMode) {
            if (gamepad1.b && !lastBState) isSlowMode = !isSlowMode;
        }
        else {
            isSlowMode = gamepad1.b;
        }
        lastBState = gamepad1.b;

        // Check which speed modifier mode to be in. Speed modifies just change joystick input
        // Pressing down on stick bumps it up a mode, slow mode bumps it down one, default is middle (1)
        final double linearSpeedModifier = DriverConstants.linearSpeedModifiers[1 + (gamepad1.left_stick_button ? 1 : 0) - (isSlowMode ? 1 : 0)];
        final double angularSpeedModifier = DriverConstants.angularSpeedModifiers[1 + (gamepad1.right_stick_button ? 1 : 0) - (isSlowMode ? 1 : 0)];

        // Left joystick does linear movement / translation (forward, backward, left, right, diagonals)
        // Right joystick does angular movement / rotation (spinning in place and arks).

        // "axial" is determined by the left stick's y-axis and controls forward and backward motion
        // "lateral" is determined by the left stick's x-axis and controls strafing motion (sideways movement)
        // "yaw" is determined by the right stick's x-axis and controls the rotation of the robot

        final double axial = -gamepad1.left_stick_y * linearSpeedModifier;
        final double lateral = gamepad1.left_stick_x * linearSpeedModifier;
        final double yaw = gamepad1.right_stick_x * angularSpeedModifier;

        double ark = gamepad1.right_stick_y * angularSpeedModifier;
        if (ark < DriverConstants.arkPowerThreshHold) ark = 0;

        // Combine joystick requests for each axis-motion to determine each wheel's power.
        double leftFrontPower = axial + lateral + yaw + ark;
        double rightFrontPower = axial - lateral - yaw - ark;
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

        final boolean shouldBrakeStoppedMotors = maxPower < DriverConstants.minPowerBrakeThreshold
                || (isSlowMode && DriverConstants.slowModeBrake);

        if (shouldBrakeStoppedMotors) {
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            leftFrontDrive.setZeroPowerBehavior(DriverConstants.wheelMotorZeroPowerBehaviorDefault);
            leftBackDrive.setZeroPowerBehavior(DriverConstants.wheelMotorZeroPowerBehaviorDefault);
            rightFrontDrive.setZeroPowerBehavior(DriverConstants.wheelMotorZeroPowerBehaviorDefault);
            rightBackDrive.setZeroPowerBehavior(DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        }

        // Up on dpad to move target position up and down to move target down
        if (gamepad1.dpad_up) targetArmPosition += ArmConstants.armMoveAmount;
        if (gamepad1.dpad_down) targetArmPosition -= ArmConstants.armMoveAmount;

        // Send power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Send target position for arm
        arm.setTargetPosition(targetArmPosition);

        // telemetry read outs
        telemetry.addData("Status", "Run Time: %s", runtime.toString());

        // drive telemetry
        telemetry.addData("Speed Modifiers", "Linear: %.0f%%, Angular: %.0f%%", linearSpeedModifier * 100, angularSpeedModifier * 100);
        telemetry.addData("Front", "Left: %4.2f, Right: %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back ", "Left: %4.2f, Right: %4.2f", leftBackPower, rightBackPower);

        // device telemetry
        telemetry.addData("Arm Position", "Current: %d, Target: %d", arm.getCurrentPosition(), targetArmPosition);

        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
        telemetry.update();

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main Remote")
public class MainRemoteOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor arm;

    private CRServo intakeServo;

    private Servo dropperServo;


    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        intakeServo = hardwareMap.get(CRServo.class, "intake");
        dropperServo = hardwareMap.get(Servo.class, "dropper");

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
        arm.setDirection(Constants.ArmConstants.armDirection);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(Constants.ArmConstants.armPower);

        int targetArmPosition = 0;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(targetArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        boolean isSlowMode = false;
        boolean lastBState = false;

        int intakeSpin = 0;
        boolean lastIntakeButtonState = false;
        boolean lastReverseIntakeButtonState = false;

        while (opModeIsActive()) {
            // Check if slow mode is enabled and check if it should toggle on or if it must be held
            if (Constants.DriverConstants.slowModeIsToggleMode) {
                if (gamepad1.b && !lastBState) isSlowMode = !isSlowMode;
            } else {
                isSlowMode = gamepad1.b;
            }
            lastBState = gamepad1.b;

            if (Constants.DriverConstants.grabberSpinIsToggleMode) {
                if (gamepad1.dpad_left && !lastIntakeButtonState)
                    intakeSpin = (intakeSpin == 1) ? 0 : 1;
                if (gamepad1.dpad_right && !lastReverseIntakeButtonState)
                    intakeSpin = (intakeSpin == -1) ? 0 : -1;

            } else {
                intakeSpin = (gamepad1.dpad_left ? 0 : 1) - (gamepad1.dpad_right ? 0 : 1);
            }
            lastIntakeButtonState = gamepad1.dpad_left;
            lastReverseIntakeButtonState = gamepad1.dpad_right;

            if (gamepad1.y) {
                dropperDown();
            }
            if (gamepad1.x) {
                dropperUp();
            }
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


            // Combine joystick requests for each axis-motion to determine each wheel's power.
            double leftFrontPower = axial + lateral + yaw + frontArk;
            double rightFrontPower = axial - lateral - yaw - frontArk;
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

            // Set the motors to resist any movement if the max power being sent to the motors is below threshold
            final boolean shouldBrakeStoppedMotors = maxPower < Constants.DriverConstants.minPowerBrakeThreshold
                    || (isSlowMode && Constants.DriverConstants.slowModeBrake);

            if (shouldBrakeStoppedMotors) {
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                leftFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
                leftBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
                rightFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
                rightBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
            }

            // Up on dpad to move target position up.
            // Down on dpad to move it down.
            if (gamepad1.dpad_up)
                targetArmPosition = Math.min(targetArmPosition + Constants.ArmConstants.armMoveAmount, Constants.ArmConstants.maxPosition);
            if (gamepad1.dpad_down)
                targetArmPosition = Math.max((targetArmPosition - Constants.ArmConstants.armMoveAmount), Constants.ArmConstants.minPosition);

            // Move to set points if triggers
            if (gamepad1.left_trigger > Constants.DriverConstants.triggerThreshold)
                targetArmPosition = Constants.ArmConstants.armUpSetPoint;
            if (gamepad1.right_trigger > Constants.DriverConstants.triggerThreshold)
                targetArmPosition = Constants.ArmConstants.armDownSetPoint;


            // Send power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Send target position for arm
            arm.setTargetPosition(targetArmPosition);

            // Send power to intake servo
            intakeServo.setPower(intakeSpin * Constants.ArmConstants.intakePower);

            // Run time telemetry, mostly just to check if the program is running all right.
            telemetry.addData("Status", "Run Time: %s", runtime.toString());

            // Drive telemetry
            telemetry.addData("Speed Modifiers", "Linear: %.0f%%, Angular: %.0f%%", linearSpeedModifier * 100, angularSpeedModifier * 100);
            telemetry.addData("Front Power", "Left: %4.2f, Right: %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back Power", "Left: %4.2f, Right: %4.2f", leftBackPower, rightBackPower);

            // Arm telemetry
            telemetry.addData("Arm Position", "Current: %d, Target: %d", arm.getCurrentPosition(), targetArmPosition);

            telemetry.addData("Intake Power", "Current: %s", intakeServo.getPower());

            telemetry.update();
        }

    }

    public void dropperUp() {
        dropperServo.setPosition(Constants.AutoConstants.UP_DROPPER_POSITION);
    }

    public void dropperDown() {
        dropperServo.setPosition(Constants.AutoConstants.DOWN_DROPPER_POSITION);
    }
}


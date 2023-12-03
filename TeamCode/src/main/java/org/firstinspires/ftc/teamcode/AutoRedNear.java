package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutoRedNear")
public class AutoRedNear extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor leftEncoder;
    private DcMotor centerEncoder;
    private DcMotor rightEncoder;

    private Servo dropperServo;
    private CRServo intakeServo;


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

        leftEncoder = hardwareMap.get(DcMotor.class, "LE");
        centerEncoder = hardwareMap.get(DcMotor.class, "CE");
        rightEncoder = hardwareMap.get(DcMotor.class, "RE");

        leftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        centerEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.get(DcMotor.class, "top arm");

        // Set arm motor mode
        arm.setDirection(Constants.ArmConstants.armDirection);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(Constants.ArmConstants.armPower);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeServo = hardwareMap.get(CRServo.class, "intake");
        dropperServo = hardwareMap.get(Servo.class, "dropper");

        telemetry.addData("Status", "Initialized").setRetained(true);
        telemetry.update();

        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {

            /** New Functions to use
             *
             * Arm Related:
             * setArmPosition(0) sets arm to target position, continues strait to next instruction. If position is not in valid range returns false
             * setArmPositionAndWait(0) sets arm to target position, waits until arm reaches that position to move to next instruction. If position is not in valid range returns false
             * resetArm() set arm back to starting position, make sure to call at end of program so MainRemoteAuto knows where arm is
             * setArmToPickupPosition() set arm to pickup position
             * setArmToDropPosition() set arm to position where tiles are dropped
             * setArmToPickupPositionAndWait() set arm to pickup position, wait for arm to reach position
             * setArmToDropPositionAndWait() set arm to position where tiles are dropped, wait for arm to reach position
             *
             * Intake Related:
             * enableIntake() start spinning intake to collect tiles
             * stopIntake() stop spinning intake
             * enableReverseIntake() reverse intake to drop tiles
             *
             * enableIntakeAndWait(1) start spinning intake to collect tiles for N seconds,
             * enableReverseAndWait(1) reverse intake to drop tiles for N seconds,
             *
             * Dropper Related:
             * dropperDown() dropper to dropping position (down)
             * dropperUp() dropper to starting position (up)
             *
             * If dropperDown() goes to wrong spot change DOWN_DROPPER_POSITION to a value you find works:
             * Constants.java
             *     public static class AutoConstants
             *         public static final double DOWN_DROPPER_POSITION = 100;
             */

            setArmPosition(100);

            telemetry.addData("Status", "Run Time: %s", runtime.toString());

            // turn left.

            driveRightInches(50);
          //  driveForwardInches(12);


            dropperDown();
        }
        resetArm();
    }

    public void driveForwardInches(double inchesToGo) {
        if (inchesToGo == 0) return;

        leftFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        leftBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);


        final double power = (inchesToGo > 0 ? 1 : -1) * Constants.AutoConstants.AUTO_DRIVE_POWER;

        final double distanceUntilStop = Math.abs(inchesToGo) - Constants.AutoConstants.MOVEMENT_BUFFER_INCHES;

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        final double startInchesL = ticksToInch(leftEncoder.getCurrentPosition());
        final double startInchesR = ticksToInch(rightEncoder.getCurrentPosition());

        double change = 0;

        telemetry.addData("Raw Encoder Inches", "Left: %4.2f, Right: %4.2f", startInchesL, startInchesR);
        telemetry.addData("Change Inches", "Change: %4.2f, Target: %4.2f", change, distanceUntilStop);

        while (change < distanceUntilStop) {
            final double currentInchesL = ticksToInch(leftEncoder.getCurrentPosition());
            final double currentInchesR = ticksToInch(rightEncoder.getCurrentPosition());

            final double changeL = currentInchesL - startInchesL;
            final double changeR = currentInchesR - startInchesR;

            change = Math.abs((changeL + changeR) / 2);

            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            telemetry.addData("Encoder Inches", "Left: %4.2f, Right: %4.2f", currentInchesL, currentInchesR);
            telemetry.addData("Change Inches", "Change: %4.2f, Target: %4.2f", change, distanceUntilStop);
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.clear();
    }

    public void driveLeftInches(double inchesToGo) {
        if (inchesToGo == 0) return;

        leftFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        leftBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);

        final double power = (inchesToGo > 0 ? 1 : -1) * Constants.AutoConstants.AUTO_DRIVE_POWER;

        final double distanceUntilStop = Math.abs(inchesToGo) - Constants.AutoConstants.MOVEMENT_BUFFER_INCHES;

        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);

        final double startInches = ticksToInch(centerEncoder.getCurrentPosition());

        double change = 0;

        telemetry.addData("Raw Encoder Inches", "Center: %4.2f", startInches);
        telemetry.addData("Change Inches", "Change: %4.2f, Target: %4.2f", change, distanceUntilStop);

        while (change < distanceUntilStop) {
            final double currentInches = ticksToInch(centerEncoder.getCurrentPosition());

            change = Math.abs(currentInches - startInches);

            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            telemetry.addData("Encoder Inches", "Center: %4.2f", currentInches);
            telemetry.addData("Change Inches", "Change: %4.2f, Target: %4.2f", change, distanceUntilStop);
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.clear();
    }



    /** Set the arms position, returns immediately, returns true if position is allowed  */
    public boolean setArmPosition(int position) {
        if (position > Constants.ArmConstants.maxPosition || position < Constants.ArmConstants.minPosition) return false;
        arm.setTargetPosition(position);
        return true;
    }

    public void setArmPositionUnsafe(int position) {
        arm.setTargetPosition(position);
    }

    /** Set the arms position only finish after arm reaches target */
    public boolean setArmPositionAndWait(int position) {
        boolean valid = setArmPosition(position);
        if (!valid) return false;
        while (Math.abs(arm.getCurrentPosition() - position) > Constants.AutoConstants.ARM_POSITION_RANGE) {
            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            telemetry.addData("Arm Position", "Current: %d, Target: %d", arm.getCurrentPosition(), position);
            telemetry.update();
        }
        telemetry.clear();
        return true;
    }

    public void resetArm() {
        setArmPosition(0);
    }

    public void setArmToPickupPosition() {
        setArmPosition(Constants.ArmConstants.armDownSetPoint);
    }

    public void setArmToDropPosition() {
        setArmPosition(Constants.ArmConstants.armUpSetPoint);
    }

    public void setArmToPickupPositionAndWait() {
        setArmPositionAndWait(Constants.ArmConstants.armDownSetPoint);
    }

    public void setArmToDropPositionAndWait() {
        setArmPositionAndWait(Constants.ArmConstants.armUpSetPoint);
    }

    public void enableIntake() {
        intakeServo.setPower(1);
    }

    public void stopIntake() {
        intakeServo.setPower(0);
    }

    public void enableReverseIntake() {
        intakeServo.setPower(-1);
    }

    public void enableIntakeAndWait(int timeSeconds) {
        enableIntake();
        double now = runtime.now(TimeUnit.SECONDS);
        final double endTime = now + timeSeconds;
        while (runtime.now(TimeUnit.SECONDS) < endTime) {
            now = runtime.now(TimeUnit.SECONDS);

            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            telemetry.addData("Intake Time Left", endTime - now);
            telemetry.update();
        }
        stopIntake();
        telemetry.clear();
    }

    public void enableReverseIntakeAndWait(int timeSeconds) {
        enableReverseIntake();
        double now = runtime.now(TimeUnit.SECONDS);
        final double endTime = now + timeSeconds;
        while (runtime.now(TimeUnit.SECONDS) < endTime) {
            now = runtime.now(TimeUnit.SECONDS);

            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            telemetry.addData("Drop Time Left", endTime - now);
            telemetry.update();
        }
        stopIntake();
        telemetry.clear();
    }

    public void dropperUp() {
        dropperServo.setPosition(Constants.AutoConstants.UP_DROPPER_POSITION);
    }

    public void dropperDown() {
        dropperServo.setPosition(Constants.AutoConstants.DOWN_DROPPER_POSITION);
    }

    public void driveRightInches(double inchesToGo) {
        if (inchesToGo == 0) return;

        leftFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        leftBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);

        final double power = (inchesToGo > 0 ? 1 : -1) * Constants.AutoConstants.AUTO_DRIVE_POWER;

        final double distanceUntilStop = Math.abs(inchesToGo) - Constants.AutoConstants.MOVEMENT_BUFFER_INCHES;

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);

        final double startInches = ticksToInch(centerEncoder.getCurrentPosition());

        double change = 0;

        telemetry.addData("Raw Encoder Inches", "Center: %4.2f", startInches);
        telemetry.addData("Change Inches", "Change: %4.2f, Target: %4.2f", change, distanceUntilStop);

        while (change < distanceUntilStop) {
            final double currentInches = ticksToInch(centerEncoder.getCurrentPosition());

            change = Math.abs(currentInches - startInches);

            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            telemetry.addData("Encoder Inches", "Center: %4.2f", currentInches);
            telemetry.addData("Change Inches", "Change: %4.2f, Target: %4.2f", change, distanceUntilStop);
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.clear();
    }
    public double ticksToInch(int ticks) {
        return (ticks * Constants.OdometryConstants.tickInMM) / 25.4;
    }

    public void driveDirectionTime(double axial, double lateral, double yaw, int time) {
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

        leftFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        leftBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightFrontDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);
        rightBackDrive.setZeroPowerBehavior(Constants.DriverConstants.wheelMotorZeroPowerBehaviorDefault);

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        double now = runtime.now(TimeUnit.SECONDS);
        final double endTime = now + time;
        while (runtime.now(TimeUnit.SECONDS) < endTime) {
            now = runtime.now(TimeUnit.SECONDS);

            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            telemetry.addData("Time Left", endTime - now);
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.clear();
    }
}
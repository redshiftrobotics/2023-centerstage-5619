package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Main Auto")
public class MainAuto extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor leftEncoder;
    private DcMotor centerEncoder;
    private DcMotor rightEncoder;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            driveForwardInches(12);
        }
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

        final double endTime = runtime.now(TimeUnit.SECONDS) + time;
        while (runtime.now(TimeUnit.SECONDS) < endTime) {
            telemetry.addData("Time Left", endTime - runtime.now(TimeUnit.SECONDS));
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
    }
}
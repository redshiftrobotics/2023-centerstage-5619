package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Main Auto")
public class MainAuto extends LinearOpMode {

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
        if (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            driveDirections(0.5, 0, 0, 1);
        }
    }

    public void driveDirections(double axial, double lateral, double yaw, int time) {
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
            telemetry.addData("Time Left", runtime.now(TimeUnit.SECONDS) - endTime);
            telemetry.update();
        }

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
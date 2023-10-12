package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Test Motor Id and Direction", group="Linear OpMode")
@Disabled
public class testMotorsIdDirection extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    final private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // get motors using set motor names
        final DcMotor leftFrontDrive  = hardwareMap.get(DcMotor.class, "FL");
        final DcMotor leftBackDrive  = hardwareMap.get(DcMotor.class, "BL");
        final DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        final DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "BR");


        // set if motor is reversed
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // test code

            //

            double leftFrontPower  = gamepad1.x ? 1.0 : 0.0;
            double leftBackPower   = gamepad1.a ? 1.0 : 0.0;
            double rightFrontPower = gamepad1.y ? 1.0 : 0.0;
            double rightBackPower  = gamepad1.b ? 1.0 : 0.0;

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}


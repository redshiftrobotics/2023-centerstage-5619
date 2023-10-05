package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Omni Drive", group="Linear OpMode")
public class driveTrain extends LinearOpMode {

  // Declare OpMode members for each of the 4 motors.
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor leftFrontDrive = null;
  private DcMotor leftBackDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor rightBackDrive = null;

  @Override
  public void runOpMode() {

    // get motors using set motor names
    leftFrontDrive  = hardwareMap.get(DcMotor.class, "FL");
    leftBackDrive  = hardwareMap.get(DcMotor.class, "BL");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
    rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

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
      double max;

      // left joystick to go forward & strafe, and right joystick to rotate.
      double axial = -gamepad1.left_stick_y;
      double lateral = gamepad1.left_stick_x;
      double yaw = gamepad1.right_stick_x;

      // Combine joystick requests for each axis-motion to determine each wheel's power.
      double leftFrontPower = axial + lateral + yaw;
      double rightFrontPower = axial - lateral - yaw;
      double leftBackPower = axial - lateral + yaw;
      double rightBackPower = axial + lateral - yaw;


      // Normalize values so no wheel power exceeds 100%
      max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
      max = Math.max(max, Math.abs(leftBackPower));
      max = Math.max(max, Math.abs(rightBackPower));

      if (max > 1.0) {
        leftFrontPower /= max;
        rightFrontPower /= max;
        leftBackPower /= max;
        rightBackPower /= max;
      }

      // Send power to wheels
      leftFrontDrive.setPower(leftFrontPower);
      rightFrontDrive.setPower(rightFrontPower);
      leftBackDrive.setPower(leftBackPower);
      rightBackDrive.setPower(rightBackPower);

      // telemetry
      telemetry.addData("Status", "Run Time: " + runtime.toString());
      telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
      telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
      telemetry.update();
    }
  }}

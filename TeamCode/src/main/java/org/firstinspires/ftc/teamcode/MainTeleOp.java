package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;

@TeleOp(name="Main Drive", group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {

  // Declare OpMode members for each of the 4 motors.
  final private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {

    // get motors using set motor names
    final DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
    final DcMotor leftBackDrive  = hardwareMap.get(DcMotor.class, "BL");
    final DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
    final DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    final DcMotor arm = hardwareMap.get(DcMotor.class, "top arm");

    int targetArmPosition = arm.getCurrentPosition();

    arm.setTargetPosition(targetArmPosition);
    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm.setPower(Constants.armPower);

    // set if motor is reversed


    // Wait for the game to start (driver presses PLAY)
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();
    runtime.reset();

    boolean isSlowMode = Constants.shouldStartInSlow;
    boolean lastBstate = false;

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      double max;

      if (Constants.slowModeIsToggleMode) {
        if (gamepad1.b && !lastBstate) isSlowMode = !isSlowMode;
      }
      else {
        isSlowMode = gamepad1.b;
      }
      lastBstate = gamepad1.b;

      // left joystick to go forward & strafe, and right joystick to rotate.
      double axial = -gamepad1.left_stick_y * Constants.moveSpeedModifiers[1 + (gamepad1.left_stick_button ? 1 : 0) - (isSlowMode ? 1 : 0)];
      double lateral = gamepad1.left_stick_x * Constants.moveSpeedModifiers[1 + (gamepad1.left_stick_button ? 1 : 0) - (isSlowMode ? 1 : 0)];
      double yaw = gamepad1.right_stick_x * Constants.spinSpeedModifiers[1 + (gamepad1.right_stick_button ? 1 : 0) - (isSlowMode ? 1 : 0)];

      if (gamepad1.dpad_down) targetArmPosition += Constants.armMoveAmount;
      if (gamepad1.dpad_down) targetArmPosition -= Constants.armMoveAmount;

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

      arm.setTargetPosition(targetArmPosition);

      // Send power to wheels
      leftFrontDrive.setPower(leftFrontPower);
      rightFrontDrive.setPower(rightFrontPower);
      leftBackDrive.setPower(leftBackPower);
      rightBackDrive.setPower(rightBackPower);

      // telemetry
      telemetry.addData("Status", "Run Time: %s", runtime.toString());
      telemetry.addData("Front Left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
      telemetry.addData("Back  Left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

      telemetry.addData("Arm   Target/Real", "%d, %d", targetArmPosition, arm.getCurrentPosition());

      telemetry.addData("Slow Mode: ", String.valueOf(isSlowMode));

      telemetry.update();

    }
  }}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Main Drive", group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {

  // Declare OpMode members for each of the 4 motors.
  final private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {

    // Get motors using set motor names
    final DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
    final DcMotor leftBackDrive  = hardwareMap.get(DcMotor.class, "BL");
    final DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
    final DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

    // Set if motor is reversed
    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    // Enable or disable braking
    leftFrontDrive.setZeroPowerBehavior(Constants.moveMotorZeroBehavior);
    leftBackDrive.setZeroPowerBehavior(Constants.moveMotorZeroBehavior);
    rightFrontDrive.setZeroPowerBehavior(Constants.moveMotorZeroBehavior);
    rightBackDrive.setZeroPowerBehavior(Constants.moveMotorZeroBehavior);

    // Set arm motor mode
    final DcMotor arm = hardwareMap.get(DcMotor.class, "top arm");
    arm.setDirection(DcMotor.Direction.FORWARD);

    int targetArmPosition = arm.getCurrentPosition();

    arm.setTargetPosition(targetArmPosition);
    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    arm.setPower(Constants.armPower);

    // Wait for the game to start (driver presses PLAY)
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();
    runtime.reset();

    telemetry.addData("Status", "Staring...");
    telemetry.update();

    // Set up slow mode variables
    boolean isSlowMode = Constants.shouldStartInSlow;
    boolean lastBState = false;

    // Run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      // Check if slow mode is enabled and check if it should toggle on or if it must be held
      // TODO make wrapper around button presses to check for keydown/keyup
      if (Constants.slowModeIsToggleMode) {
        if (gamepad1.b && !lastBState) isSlowMode = !isSlowMode;
      }
      else {
        isSlowMode = gamepad1.b;
      }
      lastBState = gamepad1.b;

      // Check which speed modifier mode to be in. Speed modifies just change joystick input
      // Pressing down on stick bumps it up a mode, slow mode bumps it down one, default is middle (1)
      final double moveSpeedModifier = Constants.moveSpeedModifiers[1 + (gamepad1.left_stick_button ? 1 : 0) - (isSlowMode ? 1 : 0)];
      final double spinSpeedModifier = Constants.spinSpeedModifiers[1 + (gamepad1.right_stick_button ? 1 : 0) - (isSlowMode ? 1 : 0)];

      // Left joystick to go forward & strafe, and right joystick to rotate.
      double axial = -gamepad1.left_stick_y * moveSpeedModifier;
      double lateral = gamepad1.left_stick_x * moveSpeedModifier;
      double yaw = gamepad1.right_stick_x * spinSpeedModifier;

      // Up on dpad to move target position up and down to move target down
      if (gamepad1.dpad_down) targetArmPosition += Constants.armMoveAmount;
      if (gamepad1.dpad_down) targetArmPosition -= Constants.armMoveAmount;

      // Combine joystick requests for each axis-motion to determine each wheel's power.
      double leftFrontPower = axial + lateral + yaw;
      double rightFrontPower = axial - lateral - yaw;
      double leftBackPower = axial - lateral + yaw;
      double rightBackPower = axial + lateral - yaw;

      // Normalize values so no wheel power exceeds 100%
      double max = Math.max(
              Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
              Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
      );

      if (max > 1.0) {
        leftFrontPower /= max;
        rightFrontPower /= max;
        leftBackPower /= max;
        rightBackPower /= max;
      }

      // Send target position for arm
      arm.setTargetPosition(targetArmPosition);

      // Send power to wheels
      leftFrontDrive.setPower(leftFrontPower);
      rightFrontDrive.setPower(rightFrontPower);
      leftBackDrive.setPower(leftBackPower);
      rightBackDrive.setPower(rightBackPower);

      // telemetry read outs
      telemetry.addData("Status", "Run Time: %s", runtime.toString());

      // drive telemetry
      telemetry.addData("Speed Modifiers", "Move: %f, Spin: %f", moveSpeedModifier, spinSpeedModifier);
      telemetry.addData("Front", "Left: %4.2f, Right: %4.2f", leftFrontPower, rightFrontPower);
      telemetry.addData("Back ", "Left: %4.2f, Right: %4.2f", leftBackPower, rightBackPower);

      // device telemetry
      telemetry.addData("Arm Position", "Current: %d, Target: %d", arm.getCurrentPosition(), targetArmPosition);

      telemetry.update();
    }
  }
}

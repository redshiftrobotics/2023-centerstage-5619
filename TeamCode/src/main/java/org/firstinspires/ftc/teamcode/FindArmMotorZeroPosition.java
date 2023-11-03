package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/** How to use:
 * 1. Put arm in your new starting position
 * 2. Use Up and Down on DPad to put arm at the original starting position (Horizontal to ground, sticking strait forward). Speed is slow to prevent accidents because limits are off.
 * 3. Press A to
 * 4. Press B to enable limits and set points. Check to make sure everything is correct. Be ready to stop when testing set points.
 * 5. Put in your new armLocationShift in Constants.ArmConstants. Old armLocationShift should be 0.
 */

@TeleOp(name = "Find Arm Motor Zero Position")
public class FindArmMotorZeroPosition extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor arm;

    @Override
    public void runOpMode() {
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

        boolean enableLimitsAndSetPoints = false;
        int armLocationShift = 0;

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: %s", runtime.toString());

            if (gamepad1.dpad_up) targetArmPosition++;
            if (gamepad1.dpad_down) targetArmPosition--;

            if (gamepad1.b) enableLimitsAndSetPoints = !enableLimitsAndSetPoints;

            if (gamepad1.a) armLocationShift = arm.getCurrentPosition();

            if (enableLimitsAndSetPoints) {
                if (targetArmPosition < Constants.ArmConstants.minPosition + armLocationShift) {
                    targetArmPosition = Constants.ArmConstants.minPosition + armLocationShift;
                }
                if (targetArmPosition > Constants.ArmConstants.maxPosition + armLocationShift) {
                    targetArmPosition = Constants.ArmConstants.maxPosition + armLocationShift;
                }

                if (gamepad1.left_trigger > Constants.DriverConstants.triggerThreshold) targetArmPosition = Constants.ArmConstants.armUpSetPoint + armLocationShift;
                if (gamepad1.right_trigger > Constants.DriverConstants.triggerThreshold) targetArmPosition = Constants.ArmConstants.armDownSetPoint + armLocationShift;
            }

            arm.setTargetPosition(targetArmPosition);

            telemetry.addData("Arm Position (D-Pad)", "Current: %d, Target: %d", arm.getCurrentPosition(), targetArmPosition);
            telemetry.addData("Limits and Set points (B)", enableLimitsAndSetPoints ? "On" : "Off");
            telemetry.addData("Arm Location Shift (A)", armLocationShift);
            telemetry.addData("Arm Position Shifted", "Current: %d, Target: %d", arm.getCurrentPosition() + armLocationShift, targetArmPosition + armLocationShift);


            telemetry.update();
        }

    }
}
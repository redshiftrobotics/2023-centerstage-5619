package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Odometry Pod")
@Disabled
public class TestOdometryPod extends LinearOpMode {
    private DcMotor motor;


    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();

        final ElapsedTime runtime = new ElapsedTime();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: %s", runtime.toString());

            final int motorPosition = motor.getCurrentPosition();

            telemetry.addData("Motor Current", motorPosition % (int) Constants.OdometryConstants.ticksInRotation);
            telemetry.addData("Motor Total Ticks", motorPosition);

            final double distanceMM = motorPosition * Constants.OdometryConstants.tickInMM;

            telemetry.addData("Motor Distance MM", distanceMM);

            final double distanceTotalInches = distanceMM / 25.4;
            final int feet = (int) distanceTotalInches / 12;
            final int inches = (int) distanceTotalInches % 12;

            telemetry.addData("Motor Distance Feet", "%s feet and %s", feet, inches);


            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "TestContinuousServo")
@Disabled
public class TestContinuousServo extends LinearOpMode {
    private CRServo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(CRServo.class, "crservo");
        servo.resetDeviceConfigurationForOpMode();

        waitForStart();

        while (opModeIsActive()) {
            servo.setPower(1);
            telemetry.addData("Servo Power", servo.getPower());
            telemetry.addData("Servo Direction", servo.getDirection());
            telemetry.update();
        }
    }
}
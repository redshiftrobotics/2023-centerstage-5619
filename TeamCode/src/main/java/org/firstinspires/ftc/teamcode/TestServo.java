package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestServo")
@Disabled
public class TestServo extends LinearOpMode {
    private Servo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.resetDeviceConfigurationForOpMode();

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(0);
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}
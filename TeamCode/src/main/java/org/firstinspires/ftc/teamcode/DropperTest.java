package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Drop Test")
@Disabled
public class DropperTest extends LinearOpMode {
    private Servo dropperServo;
    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        dropperServo = hardwareMap.get(Servo.class, "dropper");

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            if (runtime.seconds() % 5 == 0) {
                if (runtime.seconds() % 10 == 0) {
                    dropperDown();
                }
                else {
                    dropperUp();
                }
            }
        }
    }

    public void dropperUp() {
        dropperServo.setPosition(Constants.AutoConstants.UP_DROPPER_POSITION);
    }

    public void dropperDown() {
        dropperServo.setPosition(Constants.AutoConstants.DOWN_DROPPER_POSITION);
    }
}

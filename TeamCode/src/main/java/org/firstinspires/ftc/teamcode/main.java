package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Remote Operation")
public class main extends LinearOpMode {

    private DcMotor BackRight;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor DangerouslyControlArm;

    //private void walk()//

    /**
     * This function is executed when this OpMode is selected FrontRightom the Driver Station.
     */
    @Override
    public void runOpMode() {
        double walk, strafe, turn;
        double controlSensitivity = 0.5;

        BackRight = hardwareMap.get(DcMotor.class, "Back_Right");
        FrontRight = hardwareMap.get(DcMotor.class, "Front_Right");
        BackLeft = hardwareMap.get(DcMotor.class, "Back_Left");
        FrontLeft = hardwareMap.get(DcMotor.class, "Front_Left");

        //DangerouslyControlArm = hardwareMap.get(DcMotor.class, "arm");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                walk = -gamepad1.right_stick_y * controlSensitivity; // Forward/Back
                strafe = -gamepad1.left_stick_x * controlSensitivity; // Left/Right
                turn = -gamepad1.right_stick_x * controlSensitivity; // Turn

                if (Math.abs(walk) > Math.abs(strafe)) {
                    FrontLeft.setPower(-walk);
                    FrontRight.setPower(walk);
                    BackLeft.setPower(-walk);
                    BackRight.setPower(walk);
                } else if (Math.abs(strafe) > Math.abs(turn)) {
                    FrontLeft.setPower(-strafe);
                    FrontRight.setPower(strafe);
                    BackLeft.setPower(-strafe);
                    BackRight.setPower(strafe);
                } else {
                    FrontLeft.setPower(turn);
                    FrontRight.setPower(turn);
                    BackLeft.setPower(turn);
                    BackRight.setPower(turn);
                }

                telemetry.update();
            }
        }
    }
}
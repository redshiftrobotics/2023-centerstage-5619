package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ArmTick")
public class ArmTick extends LinearOpMode {
    private DcMotor arm;


    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class, "top arm");
        waitForStart();
        while (opModeIsActive()) {
            arm.setPower(0.5);
        }
    }
}
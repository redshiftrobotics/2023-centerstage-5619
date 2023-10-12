package org.firstinspires.ftc.teamcode;
//
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ArmTick")
@Disabled
public class ArmTick extends LinearOpMode {
    private DcMotor Arm;


    @Override
    public void runOpMode() {
        Arm = hardwareMap.get(DcMotor.class, "top arm");
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double TargetPosition = (gamepad1.right_stick_y * 80000) + Arm.getCurrentPosition();

                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setTargetPosition((int) TargetPosition);
                Arm.setPower(.6);
                telemetry.addData("Gamepad Input was " + gamepad1.right_stick_y + ". As a result, the motor was told to spin to" + TargetPosition + "and spun to" + Arm.getCurrentPosition(), null);
                telemetry.update();
            }
        }

    }
    /*public double ConvertBooleanToDouble(boolean bool){

        double DpadUpDown;
        if (bool){
            DpadUpDown = 1;

        } else{
            DpadUpDown = 0;
        }
        return DpadUpDown;


    }*/
}
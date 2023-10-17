package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Constants {

    public static  final double armPower = 1;
    public static final int armMoveAmount = 1;

    public static final DcMotor.ZeroPowerBehavior moveMotorZeroBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    public static final double[] moveSpeedModifiers = {0.25, 0.75, 1};
    public static final double[] spinSpeedModifiers = {0.1, 0.5, 1};

    public static final boolean shouldStartInSlow = false;
    public static final boolean slowModeIsToggleMode = true;
}

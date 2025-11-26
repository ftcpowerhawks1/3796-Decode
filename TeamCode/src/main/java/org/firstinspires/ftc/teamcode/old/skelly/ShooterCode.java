package org.firstinspires.ftc.teamcode.old.skelly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterCode{
    private DcMotor motorShoot;


    public void init(HardwareMap hwMap) {
        motorShoot = hwMap.get(DcMotor.class, "motorShoot");
        motorShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setMotorSpeed(double speed) {
        //accepts -1.0 to 1.0
        motorShoot.setPower(speed);
    }
}

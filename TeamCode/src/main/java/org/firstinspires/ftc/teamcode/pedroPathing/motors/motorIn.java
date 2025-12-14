package org.firstinspires.ftc.teamcode.pedroPathing.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class motorIn {
    private DcMotor motorIntake;
    public void init(HardwareMap hwMap){
        motorIntake = hwMap.get(DcMotor.class, "motorIntake");
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void start(){
        motorIntake.setPower(1);
    }
}

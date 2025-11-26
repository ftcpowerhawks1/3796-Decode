package org.firstinspires.ftc.teamcode.skeleton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class shooter {
    private DcMotor motorShoot;
    private Limelight3A limelight;
    double calculatedVelocity = 0;
//TODO CALCULATE CONSTANT
    static double constant = 0.1; //Need to calculate. This takes the distance and multiplies it

    public void init(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        motorShoot = hwMap.get(DcMotor.class, "motorShoot");
        motorShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void shooterVelocity(int power){
        LLResult llResult = limelight.getLatestResult();
        double ta = llResult.getTa();
        double distanceToTag = (161.1 * Math.pow(ta, -0.5858));
        calculatedVelocity = distanceToTag * constant;
        motorShoot.setPower(calculatedVelocity*power);
    }

}

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
    static double constant = 0.00297705; //Need to calculate. This takes the distance and multiplies it

    public void init(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        motorShoot = hwMap.get(DcMotor.class, "motorShoot");
    }
    public void shooterVelocity(int power){
        LLResult llResult = limelight.getLatestResult();
        double ta = llResult.getTa();
        double distanceToTag = (161.1 * Math.pow(ta, -0.5858));
        calculatedVelocity = 0.438*(Math.pow(1.00126,distanceToTag));
        motorShoot.setPower(calculatedVelocity*power);

    }

}

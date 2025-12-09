package org.firstinspires.ftc.teamcode.skeleton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class shooter2 {
    private DcMotor motorShoot;
    private Limelight3A limelight;
    double calculatedVelocity = 0;

    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        motorShoot = hwMap.get(DcMotor.class, "motorShoot");
    }

    public void ShooterVelocity(int power){

        if(Distance() > 130) {
            calculatedVelocity= ((2.88*Math.pow(10,-6)*Math.pow(Distance(),2))-(7.87*Math.pow(10,-4)*Distance())+Math.pow(5.92,-1));
        }else if(Distance() <= 130){
            calculatedVelocity= ((4.25*Math.pow(10,-5)*Math.pow(Distance(),2))-(9.32*Math.pow(10,-3)*Distance())+Math.pow(9.89,-1));
        }

        motorShoot.setPower(calculatedVelocity*power);
    }
    public double Distance() {
        LLResult llResult = limelight.getLatestResult();
        double targetOffsetAngle_Vertical = llResult.getTy();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 22.5;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 12.250;

        // distance from the target to the floor
        double goalHeightInches = 29.375;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        double distanceFromLimelightToGoalCentimeters = distanceFromLimelightToGoalInches * 2.54;

        return distanceFromLimelightToGoalCentimeters;
    }

}

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
        calculatedVelocity = 0.438*(Math.pow(1.00125,distanceToTag));
        motorShoot.setPower(calculatedVelocity*power);

    }
//Pull request test
    public void newShooterVelocity(int power){
        LLResult llResult = limelight.getLatestResult();
        calculatedVelocity = 0.438*(Math.pow(1.00125,shooterNewDistance()));
        motorShoot.setPower(calculatedVelocity*power);

        //Push request?
    }
    public double shooterNewDistance() {
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

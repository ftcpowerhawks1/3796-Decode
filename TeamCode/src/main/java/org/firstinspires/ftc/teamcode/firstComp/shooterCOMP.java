package org.firstinspires.ftc.teamcode.firstComp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class shooterCOMP {
    private DcMotorEx motorShoot;
    private Limelight3A limelight;
    static int[] distances = {65, 70, 80, 85, 95, 100, 120, 155, 185, 190, 215, 220, 305, 325, 360, 365};
    static double[] powers = {0.57, 0.54, 0.51, 0.5, 0.49, 0.49, 0.48, 0.53, 0.56, 0.55, 0.56, 0.57, 0.61, 0.63, 0.67, 0.71};

    double shootValue = 0;

    public void init(HardwareMap hwMap,int mode) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(mode);

        motorShoot = hwMap.get(DcMotorEx.class, "motorShoot");
        motorShoot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void ShooterVelocity(int power) {
        shootValue = lookupClosest(distance(),distances,powers);
        motorShoot.setPower(shootValue*power);
    }

    public double distance() {
        LLResult llResult = limelight.getLatestResult();
        if(llResult.isValid()) {
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
        }else{
            double distance = 0;
            return distance;
        }
    }
    public double getShootValue(){
        return shootValue;
    }
    public double getCurrentVelocity(){
        double currentVelocity = motorShoot.getPower();
        double rpm = motorShoot.getVelocity() * 60.0 / 112;

        return rpm;
    }
    public static double lookupClosest(double input, int[] distances, double[] powers) {

        int closestIndex = 0;
        double smallestError = Math.abs(input - distances[0]);

        for (int i = 1; i < distances.length; i++) {
            double error = Math.abs(input - distances[i]);

            if (error < smallestError) {
                smallestError = error;
                closestIndex = i;
            }
        }

        return powers[closestIndex];
    }

}




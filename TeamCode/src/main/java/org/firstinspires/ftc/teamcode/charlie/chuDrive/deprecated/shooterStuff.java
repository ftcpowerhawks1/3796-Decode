package org.firstinspires.ftc.teamcode.charlie.chuDrive.deprecated;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class shooterStuff {
    private DcMotorEx motorShoot;
    private Limelight3A limelight;
    static int[] distances = {95, 110, 125, 140, 155, 170, 185, 200, 215, 305, 340};
    static double[] powers = {1100, 1150, 1150, 1150, 1150, 1150, 1250, 1250, 1300, 1500, 1500};

    double shootValue = 0;

    public void init(HardwareMap hwMap,int mode) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(mode);
        limelight.start();

        motorShoot = hwMap.get(DcMotorEx.class, "motorShoot");
        motorShoot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void ShooterVelocity(int power) {
        shootValue = lookupClosest(distance(),distances,powers);
        motorShoot.setVelocity(shootValue*power);
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




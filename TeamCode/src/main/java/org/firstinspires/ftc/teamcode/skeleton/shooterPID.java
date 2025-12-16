package org.firstinspires.ftc.teamcode.skeleton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.old.skelly.SimplePID;

@Config
public class shooterPID {
    private DcMotor motorShoot;
    private Limelight3A limelight;
    static int[] distances = {65, 70, 80, 85, 95, 100, 120, 155, 185, 190, 215, 220, 305, 325, 360, 365};
    static double[] powers = {0.57, 0.54, 0.51, 0.5, 0.49, 0.49, 0.48, 0.53, 0.56, 0.55, 0.56, 0.57, 0.61, 0.63, 0.67, 0.71};

    //TODO CALIBRATE
    public static double p = 0.0;
    public static double i = 0.15;
    public static double d = 0.045;

    double pidShoot = 0;

    double shootValue = 0;

    SimplePID pid = new SimplePID(p, i, d);


    public void init(HardwareMap hwMap, int mode) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(mode);

        motorShoot = hwMap.get(DcMotor.class, "motorShoot");
        motorShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ShooterVelocity(int power) {
        shootValue = lookupClosest(distance(),distances,powers);
        if(power == 1){
            pidShoot = (pid.update(distance(), shootValue));
            motorShoot.setPower(pidShoot);
        }else if(power == 0){
            pidShoot = 0;
            motorShoot.setPower(0);
            pid.reset();
        }
    }

    public double getShootValue(){
        return shootValue;
    }

    public double getPidShoot(){
        return pidShoot;
    }

    public double distance() {
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




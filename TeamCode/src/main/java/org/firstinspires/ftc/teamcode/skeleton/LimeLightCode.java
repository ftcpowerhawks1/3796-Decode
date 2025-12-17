package org.firstinspires.ftc.teamcode.skeleton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PID.SimplePID;

import java.util.List;
@Config
public class LimeLightCode {
    public static double p = 0.0;
    public static double i = 0.15;
    public static double d = 0.045;
    private Limelight3A limelight;
    private DcMotor motorTurn;

    public void init(HardwareMap hwMap){
        motorTurn = hwMap.get(DcMotor.class, "motorTurn");
        motorTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public int getAprilTag() {
        LLResult llResult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
        int tagID = -1;
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            tagID = fr.getFiducialId();
        }
        return tagID;
    }

    public double getAprilDistance() {
        LLResult llResult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
        double ta = llResult.getTa();
        double distance = (161.1 * Math.pow(ta, -0.5858));

        return distance;
    }

//NOT USED ANYMORE
    public void TurretActivate() {
        LLResult llResult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();

        SimplePID pid = new SimplePID(p, i, d);
        int currentAngle = motorTurn.getCurrentPosition();
        int leftBound = 0;
        int rightBound = 1;
        if (currentAngle != 1 ) {
            if (llResult.isValid() && llResult != null) {
                double tx = llResult.getTx();
                double output = pid.update(tx, 0);

                motorTurn.setPower(output);
            } else {
                motorTurn.setPower(0);
                pid.reset();
            }
        }
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

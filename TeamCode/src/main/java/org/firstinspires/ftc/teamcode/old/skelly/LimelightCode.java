package org.firstinspires.ftc.teamcode.old.skelly;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PID.SimplePID;

import java.util.List;
//@Config
public class LimelightCode {
    //DO NOT CHANGE THESE PID VALUES FOR THE TURRET UNDER ANY COST
    public static double p = 0.0;
    public static double i = 0.15;
    public static double d = 0.045;
    //DON'T CHANGE ^^^^
    private Limelight3A limelight;
    private DcMotor motorTurn;

    public void init(HardwareMap hwMap) {
        motorTurn = hwMap.get(DcMotor.class, "motorTurn");
        motorTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight = hwMap.get(Limelight3A.class, "limelight");
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

    public void pipelineSwap(int mode) {
        if (mode == 2) {
            mode = 0;
            limelight.pipelineSwitch(mode);
        } else {
            mode += 1;
            limelight.pipelineSwitch(mode);
        }
        if (mode == 0) {
            limelight.pipelineSwitch(mode);
        } else {
            mode -= 1;
            limelight.pipelineSwitch(mode);
        }
    }
     public void turnTable() {
         double power = 0;
//TODO NEED TO TUNE

         SimplePID pid = new SimplePID(p, i, d);

         LLResult llResult = limelight.getLatestResult();
         List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();

         if (llResult.isValid()&& llResult != null) {
             double tx = llResult.getTx();
             double output = pid.update(tx, 0);

             motorTurn.setPower(output);
         }else{
             motorTurn.setPower(0);
             pid.reset();
         }

     }
}


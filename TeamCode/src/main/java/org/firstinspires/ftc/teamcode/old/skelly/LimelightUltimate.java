package org.firstinspires.ftc.teamcode.old.skelly;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

public class LimelightUltimate extends OpMode {
    private Limelight3A limelight;

    int mode = -1;

    @Override
    public void init() {
        //name of limelight in driver hub
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    @Override
    public void start() {
        //starts the limelight (it draws alot of power)
        limelight.start();
        //changes pipeline
        limelight.pipelineSwitch(0);
    }

    @Override
    public void loop() {
        //Sets up Fiducial Results
        LLResult llResult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();

        //Relays Tx and distance(cm) if AprilTag was detected
        if (llResult != null && llResult.isValid()) {
            telemetry.addData("Status", "AprilTag(s) detected");

            //Relays Tx
            telemetry.addData("Tx", "%.1f", llResult.getTx());

            //Retrieves TotalArea(Ta) and multiplies it using a calculated constant
            double ta = llResult.getTa();
            double distance = (161.1*Math.pow(ta,-0.5858));

            /* Constants were calculated by documenting Ta values at varying intervals from the AprilTag (50cm,100cm,150cm,200cm etc.).
            We then plugged those values into desmos to get the coefficient (161.1) and the power (-0.5858) these are the values that work for us. */

            telemetry.addData("Distance(cm)", distance);

        } else {
            telemetry.addData("Status", "No AprilTags deteccted");
        }

        //Changes Pipeline on a button Press
        telemetry.addData("Pipeline", mode);
        if (gamepad1.dpadUpWasPressed()) {
            if (mode == 2) {
                mode = 0;
                limelight.pipelineSwitch(mode);
            } else {
                mode += 1;
                limelight.pipelineSwitch(mode);
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            if (mode == 0) {
                limelight.pipelineSwitch(mode);
            } else {
                mode -= 1;
                limelight.pipelineSwitch(mode);
            }
        }

        //Retrieves the tagID from the AprilTag detected
        int tagID = -1;
        for(LLResultTypes.FiducialResult fr : fiducialResults){
            telemetry.addData("ID", "%d", fr.getFiducialId());
            tagID = fr.getFiducialId();
        }


    }
}


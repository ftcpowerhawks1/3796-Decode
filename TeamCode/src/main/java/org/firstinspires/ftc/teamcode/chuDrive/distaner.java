package org.firstinspires.ftc.teamcode.chuDrive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class distaner extends OpMode {
    private Limelight3A limelight;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(2);
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if(llResult.isValid()){
            telemetry.addLine("yes");
        } else {
            telemetry.addLine("hell no");
        }
    }
}

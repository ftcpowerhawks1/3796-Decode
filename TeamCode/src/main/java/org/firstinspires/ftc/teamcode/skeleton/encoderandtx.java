package org.firstinspires.ftc.teamcode.skeleton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

//@TeleOp
public class encoderandtx extends OpMode {
    private Limelight3A limelight;
    private DcMotor motorTurn;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        motorTurn = hardwareMap.get(DcMotor.class, "motorTurn");
        motorTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();

        double tx = llResult.getTx();

        double TICKS_PER_REV = 2150.8;
        double DEGREES_PER_TICK = 360 / TICKS_PER_REV;
        int currentPos = motorTurn.getCurrentPosition();

        double txToTicks = tx / (DEGREES_PER_TICK);

        telemetry.addData("Curretnt",currentPos);
        telemetry.addData("Ticks", txToTicks);
        telemetry.addData("Tx",tx);

        motorTurn.setTargetPosition(currentPos-(int) txToTicks);

    if(currentPos >= -350 && currentPos <= 350) {
        if (txToTicks > 0) {
            motorTurn.setPower(0.5);
        } else if (txToTicks < 0) {
            motorTurn.setPower(-0.5);
        }
    }else{
        motorTurn.setPower(0);
    }
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            telemetry.addData("ID", "%d", fr.getFiducialId());
        }

    }
}

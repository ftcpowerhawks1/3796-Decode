package org.firstinspires.ftc.teamcode.skeleton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp
public class encoderandtxahootmove extends OpMode {
    private Limelight3A limelight;
    private DcMotor motorTurn,motorShoot,motorIntake;
    int lastPosition = 0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        motorTurn = hardwareMap.get(DcMotor.class, "motorTurn");
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        motorShoot = hardwareMap.get(DcMotor.class, "motorShoot");

        motorTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        double txToTicks = (int)(tx / (DEGREES_PER_TICK));


        telemetry.addData("Curretnt",currentPos);
        telemetry.addData("Ticks", txToTicks);
        telemetry.addData("Tx",tx);

        if (txToTicks > 10 || txToTicks < -10) {
            motorTurn.setTargetPosition(currentPos-(int) txToTicks);
            lastPosition = (currentPos-(int) txToTicks);
        }else{
            motorTurn.setTargetPosition(lastPosition);
        }
        

    if(currentPos >= -350 && currentPos <= 350) {
        if (txToTicks > 0) {
            motorTurn.setPower(0.8);
        } else if (txToTicks < 0) {
            motorTurn.setPower(-0.8);
        }
    }else if(currentPos > 350 && txToTicks > 0){
        motorTurn.setPower(-0.8);
    }else if(currentPos < 350 && txToTicks < 0){
        motorTurn.setPower(0.8);
    }else{
        motorTurn.setPower(0);
    }

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            telemetry.addData("ID", "%d", fr.getFiducialId());
        }
        if(gamepad1.a){
            motorIntake.setPower(1);
        }else if(gamepad1.b){
            motorIntake.setPower(-1);
        }else{
            motorIntake.setPower(0);
        }
        if(gamepad1.right_trigger>0.05){
            motorShoot.setPower(0.67);
        }else{
            motorShoot.setPower(0);
        }
    }
}

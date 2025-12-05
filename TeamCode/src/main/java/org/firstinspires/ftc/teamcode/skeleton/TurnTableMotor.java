package org.firstinspires.ftc.teamcode.skeleton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;


public class TurnTableMotor {
    private Limelight3A limelight;
    private DcMotor motorTurn;
    double txToTicks = 0;
    double tx = 0;

    int leftBound = -350; //Units:Ticks
    int rightBound = 350; //Units:Ticks
    double motorPower = 0.7;
    double TICKS_PER_REV = 2150.8; //19.2:1 Motor
    double DEGREES_PER_TICK = 360 / TICKS_PER_REV; //19.2:1 Motor
    int currentPos = 0;

    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();


        motorTurn = hwMap.get(DcMotor.class,"motorTurn");
        motorTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTurn.setTargetPosition(0);
        motorTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void track() {
        LLResult llResult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();

        motorTurn.setTargetPosition((currentPos - (int) txToTicks));

        tx = llResult.getTx();

        currentPos = motorTurn.getCurrentPosition();

        txToTicks = (int) (tx / (DEGREES_PER_TICK)); //(degree)/(degree/ticks)


            if (currentPos >= leftBound && currentPos <= rightBound) {
                if (txToTicks > 0) {
                    motorTurn.setPower(motorPower);
                } else if (txToTicks < 0) {
                    motorTurn.setPower(-motorPower);
                }
            } else if (currentPos > rightBound && txToTicks > 0) {
                motorTurn.setPower(-motorPower);
            } else if (currentPos < leftBound && txToTicks < 0) {
                motorTurn.setPower(motorPower);
            } else {
                motorTurn.setPower(0);
            }

    }

}

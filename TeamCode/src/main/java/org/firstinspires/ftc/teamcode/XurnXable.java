package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class XurnXable {
    private Limelight3A limelight;
    private DcMotor motorTurn;
    double txToTicks = 0;
    double tx = 0;

    int leftBound = -350; //Units:Ticks
    int rightBound = 350; //Units:Ticks
    double motorPower = 0.5;
    double TICKS_PER_REV = 2150.8; //19.2:1 Motor
    double DEGREES_PER_TICK = 360 / TICKS_PER_REV; //19.2:1 Motor
    int currentPos = 0;
    double seconds = 0;

    Timer timer = new Timer();


    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        motorTurn = hwMap.get(DcMotor.class, "motorTurn");
        motorTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTurn.setTargetPosition(0);
        motorTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
/*if(seconds > 10){
                if (currentPos > 0) {
                    motorTurn.setTargetPosition(0);
                    motorTurn.setPower(motorPower);
                    timer.resetTimer();
                    seconds = 0;

                } else if (currentPos < 0) {
                    motorTurn.setTargetPosition(0);
                    motorTurn.setPower(-motorPower);
                    timer.resetTimer();
                    seconds = 0;
                }
            }

    */



    public void track() {
        LLResult llResult = limelight.getLatestResult();
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
            scan();
        }



    }
    private void scan(){
        LLResult llResult = limelight.getLatestResult();
        while(!llResult.isValid()) {
            if (currentPos > rightBound) {
                motorTurn.setPower(-0.3);
                motorTurn.setTargetPosition(leftBound);
            } else if (currentPos < leftBound) {
                motorTurn.setPower(0.3);
                motorTurn.setTargetPosition(rightBound);
            }
            if (llResult.isValid()) {
                motorTurn.setTargetPosition((currentPos - (int) txToTicks));
                break;
            }
        }
    }
}

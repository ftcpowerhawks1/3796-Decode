package org.firstinspires.ftc.teamcode.skeleton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class TurnTableMotor {
    private DcMotor motorTurn;
    private Limelight3A limelight;

    static int leftBound = -350; //Units:Ticks
    static int rightBound = 350; //Units:Ticks
    static double motorPower = 0.7;

    public void init(HardwareMap hwMap) {
        motorTurn = hwMap.get(DcMotor.class, "motorTurn");
        motorTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
    }

    public void track() {
        LLResult llResult = limelight.getLatestResult();

        double tx = llResult.getTx();

        double TICKS_PER_REV = 2150.8; //19.2:1 Motor
        double DEGREES_PER_TICK = 360 / TICKS_PER_REV; //19.2:1 Motor
        int currentPos = motorTurn.getCurrentPosition();

        double txToTicks = (int) (tx / (DEGREES_PER_TICK)); //(degree)/(degree/ticks)
        motorTurn.setTargetPosition(currentPos - (int) txToTicks);
        motorTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

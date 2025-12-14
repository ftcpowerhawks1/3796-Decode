package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class XurnXable {

    private Limelight3A limelight;
    private DcMotor motorTurn;

    // Motor / encoder constants
    private final double TICKS_PER_REV = 2150.8; // 19.2:1 motor
    private final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;

    // Bounds (ticks)
    private final int leftBound = -350;
    private final int rightBound = 350;

    private final double motorPower = 0.5;
    private final double scanPower = 0.25;

    // Scan state
    private int scanTarget = rightBound;

    public void init(HardwareMap hwMap) {

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        motorTurn = hwMap.get(DcMotor.class, "motorTurn");
        motorTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTurn.setTargetPosition(0);
        motorTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTurn.setPower(motorPower);
    }

    public void track() {

        LLResult llResult = limelight.getLatestResult();
        int currentPos = motorTurn.getCurrentPosition();

        // ---------------- TAG FOUND ----------------
        if (llResult != null && llResult.isValid()) {

            double tx = llResult.getTx();

            // Deadband to prevent jitter
            if (Math.abs(tx) < 1.0) {
                motorTurn.setPower(0);
                return;
            }

            // Convert angle error to encoder ticks
            int txToTicks = (int) (tx / DEGREES_PER_TICK);

            // Compute target position
            int targetPos = currentPos - txToTicks;

            // Clamp to bounds
            targetPos = Math.max(leftBound, Math.min(rightBound, targetPos));

            motorTurn.setTargetPosition(targetPos);
            motorTurn.setPower(motorPower);

        }
        // ---------------- TAG NOT FOUND ----------------
        else {
            scan();
        }
    }

    private void scan() {

        // Only choose a new scan target once the motor reaches the current one
        if (!motorTurn.isBusy()) {

            scanTarget = (scanTarget == rightBound) ? leftBound : rightBound;

            motorTurn.setTargetPosition(scanTarget);
            motorTurn.setPower(scanPower);
        }
    }
}

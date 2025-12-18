package org.firstinspires.ftc.teamcode.flywheelChuckler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@TeleOp
public class chucklerTester extends OpMode {
    public DcMotorEx motorShoot;
    public double highVelocity = 5000;
    public double lowVelocity = 1000;
    double curTargetVelocity = highVelocity;
    double F, P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;
    @Override
    public void init() {
        motorShoot = hardwareMap.get(DcMotorEx.class, "motorShoot");
        motorShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShoot.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        motorShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        /*
        Get gamepad commands
        Set Target Velocity
        Update Telemetry
         */
        if(gamepad1.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }
        if(gamepad1.xWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad1.dpadLeftWasPressed()){
            F += stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }
        if(gamepad2.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        motorShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        motorShoot.setVelocity(curTargetVelocity);
        double curVelocity = motorShoot.getVelocity();
        double error = curTargetVelocity - curVelocity;
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity","%.2f", curVelocity);
        telemetry.addData("Error","%.2f", error);
        telemetry.addLine(" -------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}

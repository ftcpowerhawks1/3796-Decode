package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ShootandIntake extends OpMode {

    double powerIn = 0;
    double powerOut = 0;
    private DcMotor motorIn, motorOut;

    @Override
    public void init() {

        hardwareMap.get(DcMotor.class, "motorIn");
        motorIn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hardwareMap.get(DcMotor.class, "motorOut");
        motorOut.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            powerIn += 0.1;
        } else if (gamepad1.dpadDownWasPressed()) {
            powerIn -= 0.1;
        }
        if (gamepad1.yWasPressed()) {
            powerOut += 0.1;
        } else if (gamepad1.aWasPressed()) {
            powerOut -= 0.1;
        }
        telemetry.addData("Intake Power", powerIn);
        telemetry.addData("Shooter Power", powerOut);

        if (gamepad1.right_trigger > 0.1) {
            motorOut.setPower(powerOut);
        }
        if (gamepad1.left_trigger > 0.1) {
            motorIn.setPower(powerIn);
        }
    }
}

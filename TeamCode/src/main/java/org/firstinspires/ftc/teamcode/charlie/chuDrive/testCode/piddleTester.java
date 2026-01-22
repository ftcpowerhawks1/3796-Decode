package org.firstinspires.ftc.teamcode.charlie.chuDrive.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import kotlin.internal.ContractsDsl;

@Config
@TeleOp
public class piddleTester extends OpMode {
    final static double kP = 1.092233;
    final static double kI = 0.10922333;
    final static double kD = 0;
    final static double kF = 10.9223333;
    private DcMotorEx motorShoot;
    @Override
    public void init() {
        motorShoot = hardwareMap.get(DcMotorEx.class, "motorShoot");
        motorShoot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorShoot.setVelocityPIDFCoefficients(kP,kI,kD,kF);
        motorShoot.setPositionPIDFCoefficients(5.0);
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger >= 0.2){
            motorShoot.setVelocity(1200);
        } else {
            motorShoot.setVelocity(0);
        }
        telemetry.addData("Velo", motorShoot.getVelocity());
    }
}

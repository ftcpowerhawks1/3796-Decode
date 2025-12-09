package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.skeleton.LimeLightCode;
import org.firstinspires.ftc.teamcode.skeleton.TurnTableMotor;

@TeleOp
public class CHARLIEUSETHIS extends OpMode {
    double power = 0;
    private DcMotor motorShoot;
    private DcMotor motorIntake;
    LimeLightCode limeLightCode = new LimeLightCode();
    TurnTableMotor turnTableMotor = new TurnTableMotor();
    @Override
    public void init() {
        limeLightCode.init(hardwareMap);
        motorShoot = hardwareMap.get(DcMotor.class, "motorShoot");
        motorShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnTableMotor.init(hardwareMap);
    }

    @Override
    public void loop() {
        turnTableMotor.track();

        if(gamepad1.yWasPressed()){
            power += 0.1;
        }else if(gamepad1.aWasPressed()){
            power -= 0.1;
        } else if (gamepad1.bWasPressed()) {
            power += 0.05;
        } else if (gamepad1.xWasPressed()) {
            power -= 0.05;
        }else if(gamepad1.dpadUpWasPressed()){
            power += 0.01;
        }else if (gamepad1.dpadDownWasPressed()){
            power -= 0.01;
        }
        telemetry.addLine("Y and A change by .1");
        telemetry.addLine("x and B change by .05");
        telemetry.addLine("DpadUp and DpadDown change by .01");

        telemetry.addData("Distance(cm)",limeLightCode.shooterNewDistance());
        telemetry.addData("Power", power);
        motorShoot.setPower(power);
        motorIntake.setPower(gamepad1.left_trigger);
    }

}

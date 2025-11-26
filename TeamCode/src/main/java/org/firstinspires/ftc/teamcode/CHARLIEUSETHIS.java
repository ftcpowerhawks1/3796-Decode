package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.skeleton.LimeLightCode;

@TeleOp
public class CHARLIEUSETHIS extends OpMode {
    double power = 0;
    LimeLightCode limeLightCode = new LimeLightCode();
    @Override
    public void init() {
        limeLightCode.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.y){
            power += 0.1;
        }else if(gamepad1.a){
            power -= 0.1;
        } else if (gamepad1.b) {
            power += 0.05;
        } else if (gamepad1.x) {
            power -= 0.05;
        }else if(gamepad1.dpadUpWasPressed()){
            power += 0.01;
        }else if (gamepad1.dpadDownWasPressed()){
            power -= 0.01;
        }
        telemetry.addLine("Y and A change by .1");
        telemetry.addLine("x and B change by .05");
        telemetry.addLine("DpadUp and DpadDown change by .01");

        telemetry.addData("Distance(cm)",limeLightCode.getAprilDistance());
        telemetry.addData("Power", power);
    }
}

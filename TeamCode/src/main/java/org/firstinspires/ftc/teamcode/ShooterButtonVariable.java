package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.skelly.MotorCode;

@TeleOp
public class ShooterButtonVariable extends OpMode {
    MotorCode shooter = new MotorCode();

    double velocity;

    @Override
    public void init() {
        shooter.init(hardwareMap);
        velocity = 0;
    }

    @Override
    public void loop() {
        telemetry.addData("Velocity", velocity);

        if(gamepad1.right_trigger > 0.1){
            shooter.setMotorSpeed(velocity);
        }else{
            shooter.setMotorSpeed(0);
        }
        if(gamepad1.aWasPressed()){
            velocity += 0.1;
        }

        if(gamepad1.bWasPressed()){
            velocity += -0.1;
        }
    }
}

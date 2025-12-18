package org.firstinspires.ftc.teamcode.PID;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class ShooterPIDTune extends OpMode {
    shooterPID shooter = new shooterPID();
    Timer timer = new Timer();
    double seconds = 0;

    @Override
    public void init() {
        shooter.init(hardwareMap, 2);
    }

    @Override
    public void loop() {
        seconds = timer.getElapsedTimeSeconds();
        telemetry.addData("Time(s)",seconds);

        telemetry.addData("Distance", shooter.distance());
        telemetry.addData("Expected Velocity",shooter.getShootValue());
        telemetry.addData("Current Velocity",shooter.currentVelocity());

        if(gamepad1.right_trigger > 0.05){
            shooter.ShooterVelocity(1);
        }else{
            shooter.ShooterVelocity(0);
        }


    }
}

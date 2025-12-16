package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class timerTest extends OpMode {
    public Timer timer;

    @Override
    public void init() {
        timer = new Timer();
    }

    @Override
    public void loop() {
        telemetry.addData("Time", timer.getElapsedTimeSeconds());

    }
}

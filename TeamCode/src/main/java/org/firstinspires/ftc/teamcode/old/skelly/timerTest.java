package org.firstinspires.ftc.teamcode.old.skelly;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class timerTest extends OpMode {
    Timer timer = new Timer();

    double seconds = 0;

    @Override
    public void init() {

    }


    @Override
    public void loop(){
            seconds = timer.getElapsedTimeSeconds();
        if(seconds > 5){
            telemetry.addLine("TIMER RESET");
            timer.resetTimer();
            seconds=0;
        }
    }
}

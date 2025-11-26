package org.firstinspires.ftc.teamcode.old.skelly;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SimplePID {
    public static double kP, kI, kD;
    double integral = 0;
    double lastError = 0;
    long lastTime = System.nanoTime();

    public SimplePID(double p, double i, double d){
        kP = p;
        kI = i;
        kD = d;
    }

    public double update(double current, double target){
        double error = target - current;

        long now = System.nanoTime();
        double dt = (now-lastTime) / 1e9;
        lastTime = now;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        return  kP * error + kI * integral + kD * derivative;
    }

    public void reset(){
        integral = 0;
        lastError = 0;
        lastTime = System.nanoTime();
    }
}

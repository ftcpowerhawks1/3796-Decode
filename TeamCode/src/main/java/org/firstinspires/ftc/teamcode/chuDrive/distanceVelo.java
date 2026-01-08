package org.firstinspires.ftc.teamcode.chuDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


//@TeleOp
public class distanceVelo extends OpMode {
    private DcMotorEx motorShoot;
    private DcMotor motorIntake;
    double velocity = 0;
    shooterStuff shoot = new shooterStuff();
    mecanumDrive drive = new mecanumDrive();
    @Override
    public void init() {
        drive.init(hardwareMap);
        shoot.init(hardwareMap,2);
        motorShoot = hardwareMap.get(DcMotorEx.class, "motorShoot");
        motorShoot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorShoot.setVelocityPIDFCoefficients(1.092233, 0.10922333, 0, 10.9223333);
        motorShoot.setPositionPIDFCoefficients(5.0);
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
    }

    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;
        drive.drive(forward,strafe,rotate);
        double distance = shoot.distance();
        if(gamepad1.aWasPressed()){
            velocity = velocity - 50;
        } else if(gamepad1.yWasPressed()){
            velocity = velocity + 50;
        }
        motorShoot.setVelocity(velocity);
        motorIntake.setPower(gamepad1.right_trigger);
        double actual = motorShoot.getVelocity();
        telemetry.addData("Target Velo", velocity);
        telemetry.addData("Actual", actual);
        telemetry.addData("dis", distance);

    }
}

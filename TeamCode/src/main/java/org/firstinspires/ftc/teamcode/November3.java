package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.skeleton.TurnTableMotor;
import org.firstinspires.ftc.teamcode.skeleton.MecanumDriveCode;
import org.firstinspires.ftc.teamcode.skeleton.shooter;

@TeleOp
public class November3 extends OpMode{
    TurnTableMotor turnTableMotor = new TurnTableMotor();
    MecanumDriveCode drive = new MecanumDriveCode();
    shooter shoot = new shooter();
    private DcMotor motorIntake;

    @Override
    public void init() {
        motorIntake= hardwareMap.get(DcMotor.class, "motorIntake");
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.init(hardwareMap);
        shoot.init(hardwareMap);
        turnTableMotor.init(hardwareMap);
    }

    @Override
    public void loop() {
//DRIVE
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double maxSpeed = 1.0;

        drive.drive(forward,strafe,rotate,maxSpeed);

//TURRET
        turnTableMotor.track();

//SHOOTER
        if(gamepad2.right_trigger > 0.05){
            shoot.shooterVelocity(1);
        }else{
            shoot.shooterVelocity(0);
        }

//INTAKE
        if(gamepad1.left_trigger > 0.05){
            motorIntake.setPower(-1);
        }else if(gamepad1.right_trigger > 0.05){
            motorIntake.setPower(1);
        }else{
            motorIntake.setPower(0);
        }

    }

}
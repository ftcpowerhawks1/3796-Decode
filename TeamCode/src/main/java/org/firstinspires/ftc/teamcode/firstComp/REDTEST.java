package org.firstinspires.ftc.teamcode.firstComp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.skeleton.MecanumDriveCode;
import org.firstinspires.ftc.teamcode.skeleton.TurnTableMotor;

//@TeleOp
public class REDTEST extends OpMode{
    TurnTableMotor turnTableMotor = new TurnTableMotor();
    MecanumDriveCode drive = new MecanumDriveCode();
    shooterCOMP shoot = new shooterCOMP();


    private DcMotor motorIntake;

    @Override
    public void init() {
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");

        drive.init(hardwareMap);

        //MODE = 1 BLUE || MODE = 2 RED
        shoot.init(hardwareMap,2);

        turnTableMotor.init(hardwareMap);
    }

    @Override
    public void loop() {
//DRIVE
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double maxSpeed = 1.0;

        drive.drive(forward,strafe,rotate,maxSpeed);

//TURRET
        turnTableMotor.track();

        telemetry.addData("Distance (cm)",shoot.distance());

        telemetry.addData("Expected Percentage", shoot.getShootValue());
        telemetry.addData("Current Velocity",shoot.getCurrentVelocity());

//SHOOTER
        if(gamepad1.right_trigger > 0.05){
            shoot.ShooterVelocity(1);
        }else{
            shoot.ShooterVelocity(0);
        }

//INTAKE
        if(gamepad1.left_trigger > 0.05){
            motorIntake.setPower(-1);
        }else if(gamepad1.yWasPressed()  || gamepad2.left_trigger > 0.05){
            motorIntake.setPower(1);
        }else{
            motorIntake.setPower(0);
        }

    }

}
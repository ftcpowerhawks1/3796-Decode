package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.skeleton.MecanumDriveCodePinpoint;
import org.firstinspires.ftc.teamcode.skeleton.TurnTableMotor;
import org.firstinspires.ftc.teamcode.skeleton.shooter2;

//@TeleOp
public class December1 extends OpMode{
    TurnTableMotor turnTableMotor = new TurnTableMotor();
    MecanumDriveCodePinpoint drive = new MecanumDriveCodePinpoint();
    shooter2 shoot2 = new shooter2();

    private DcMotor motorIntake;

    @Override
    public void init() {
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.init(hardwareMap);
        shoot2.init(hardwareMap);
        turnTableMotor.init(hardwareMap);

    }

    @Override
    public void loop() {
//DRIVE
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;
        double maxSpeed = 1.0;

        drive.drive(forward,strafe,rotate,maxSpeed);

//TURRET
        turnTableMotor.track();
        telemetry.addData("Dist",shoot2.distance());
        telemetry.addData("Power", shoot2.powerLevel(shoot2.distance()));

//SHOOTER
        if(gamepad2.right_trigger > 0.05){
            shoot2.ShooterVelocity(1);
        }else{
            shoot2.ShooterVelocity(0);
        }

//INTAKE
        if(gamepad1.left_trigger > 0.05){
            motorIntake.setPower(-1);
        }else if(gamepad1.right_trigger > 0.05 || gamepad2.left_trigger > 0.05){
            motorIntake.setPower(1);
        }else{
            motorIntake.setPower(0);
        }

    }

}
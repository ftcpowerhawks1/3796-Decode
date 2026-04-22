package org.firstinspires.ftc.teamcode.charlie.chuDrive.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.charlie.chuDrive.packages.aimCode;
import org.firstinspires.ftc.teamcode.charlie.chuDrive.packages.mecanumDrive;

@TeleOp
public class blueAnureism extends OpMode {
    // Innie thing and outie thing motors
    private DcMotor intake;     // Pulls artifacts in
    // Flywheel shooter
    // Drivetrain motors
    double driveMult = 1;
    double inMult = 1;
    mecanumDrive drive = new mecanumDrive();
    aimCode aim = new aimCode();
    shooterStuff shoot = new shooterStuff();
    @Override
    public void init() {
        int shootMode = 1;
        telemetry.addData("Red is 2, Blue is 1", shootMode);

        // ---- Mechanism Motors ----
        intake = hardwareMap.get(DcMotor.class, "motorIntake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- Drivetrain Motors ----
        drive.init(hardwareMap);
        // ---- Aim Motor ----
        aim.init(hardwareMap);
        // ---- Shoot ----
        shoot.init(hardwareMap, shootMode);
    }

    @Override
    public void loop() {
        // Start of Intake/Turn/Shoot section
        // First, intake :|
        if(gamepad2.xWasPressed()){
            inMult = inMult - 0.1;
        } else if(gamepad2.bWasPressed()){
            inMult = inMult + 0.1;
        }
        intake.setPower(gamepad2.left_stick_y+gamepad1.right_trigger-gamepad1.left_trigger * inMult);

        telemetry.addData("DriveMult", driveMult);
        telemetry.addData("InMult", inMult);
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        drive.drive(forward,strafe,rotate);
        aim.track();
        if(gamepad2.right_trigger > 0.05){
            shoot.ShooterVelocity(1);
        }else{
            shoot.ShooterVelocity(0);
        }
    }
}

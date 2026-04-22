package org.firstinspires.ftc.teamcode.charlie.chuDrive.teleOPs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.charlie.chuDrive.packages.aimCode;
import org.firstinspires.ftc.teamcode.charlie.chuDrive.packages.mecanumDrive;
import org.firstinspires.ftc.teamcode.charlie.chuDrive.packages.shooterPIDF;

@TeleOp
public class    newPIDFRed extends OpMode {
    // Innie thing and out thing motors
    private CRServo intakeFront;     // Pulls artifacts in
    private CRServo intakeBack;
    // Flywheel shooter
    // Drivetrain motors
    mecanumDrive drive = new mecanumDrive();
    aimCode aim = new aimCode();
    shooterPIDF shoot = new shooterPIDF();
    @Override
    public void init() {
        int shootMode = 2;
        telemetry.addData("Red is 2, Blue is 1", shootMode);

        // ---- Mechanism Motors ----
        intakeFront = hardwareMap.get(CRServo.class, "frontServo");
        intakeBack = hardwareMap.get(CRServo.class, "backServo");

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
        telemetry.addData("dist", shoot.distance());
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;
        telemetry.addData("trig", gamepad1.right_trigger - gamepad1.left_trigger);
        intakeFront.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        if(gamepad1.right_bumper){
            intakeBack.setPower(-1);
        } else if (gamepad1.left_bumper) {
            intakeBack.setPower(1);
            
        } else {
            intakeBack.setPower(0);
        }
        drive.drive(forward,strafe,rotate);
        aim.track();
        if(gamepad2.right_trigger > 0.05){
            shoot.ShooterVelocity(1);
        }else{
            shoot.ShooterVelocity(0);
        }
    }
}

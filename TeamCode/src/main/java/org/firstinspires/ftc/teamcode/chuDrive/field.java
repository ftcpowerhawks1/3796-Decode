package org.firstinspires.ftc.teamcode.chuDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class field extends OpMode {
    // Innie thing and outie thing motors
    private DcMotor intake;     // Pulls artifacts in
    // Flywheel shooter
    // Drivetrain motors
    double driveMult = 1;
    double inMult = 1;
    mecFieldCent drive = new mecFieldCent();
    aimCode aim = new aimCode();
    shooterStuff shoot = new shooterStuff();
    @Override
    public void init() {
        drive.resetHeading();
        int shootMode = 2;
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
        intake.setPower(-gamepad2.left_stick_y * inMult);
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
        telemetry.addData("heading", drive.getHeading());
    }
}

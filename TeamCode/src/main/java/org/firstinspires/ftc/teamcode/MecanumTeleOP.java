package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.skelly.MecanumDriveCode;

@TeleOp
public class MecanumTeleOP extends OpMode {
    MecanumDriveCode drive = new MecanumDriveCode();

    double forward, strafe, rotate;
    double speed = 1.0;
    int mode = -1;

    @Override
    public void init() {
        drive.init(hardwareMap);
        telemetry.addData("Mode", "null");

    }

    @Override
    public void loop() {

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        telemetry.addData("Speed", speed);

        if (mode == -1) {
            telemetry.addData("Mode", "null");
        } else if (mode == 0) {
            telemetry.addData("Mode", "Robot Oriented");
            drive.drive(forward, strafe, rotate, speed);
        } else if (mode == 1) {
            telemetry.addData("Mode", "Field Oriented");
            drive.driveFieldRelative(forward, strafe, rotate, speed);
        }
        if (gamepad1.dpad_down) {
            mode = 0;
        } else if (gamepad1.dpad_up) {
            mode = 1;
        }
        if (speed > 1) {
            speed = 1;
        } else if (speed < 0) {
            speed = 0;
        } else {
            if (gamepad1.leftBumperWasPressed()) {
                speed += -0.1;
                telemetry.addData("Speed", speed);
            } else if (gamepad1.rightBumperWasPressed()) {
                speed += 0.1;
                telemetry.addData("Speed", speed);
            }
        }
    }
}

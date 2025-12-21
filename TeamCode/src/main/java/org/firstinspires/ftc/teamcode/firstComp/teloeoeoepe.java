package org.firstinspires.ftc.teamcode.firstComp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.skeleton.MecanumDriveCodePinpoint;
@TeleOp
public class teloeoeoepe extends OpMode {
    MecanumDriveCodePinpoint drive = new MecanumDriveCodePinpoint();

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward,strafe,rotate);
        drive.robotPose();
    }
}

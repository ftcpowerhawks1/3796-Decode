package org.firstinspires.ftc.teamcode.charlie.chuDrive.testCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class moTest extends OpMode {
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            backLeftMotor.setPower(gamepad1.right_trigger);
        } else if (gamepad1.b){
            backRightMotor.setPower(gamepad1.right_trigger);
        } else if (gamepad1.x){
            frontLeftMotor.setPower(gamepad1.right_trigger);
        } else if (gamepad1.y){
            frontRightMotor.setPower(gamepad1.right_trigger);
        }
    }
}

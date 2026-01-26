package org.firstinspires.ftc.teamcode.nicholas.testcodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class servointake extends OpMode {

    private CRServo frontServo;
    private CRServo backServo;
    private DcMotorEx motorShoot;

    @Override
    public void init() {
    frontServo = hardwareMap.get(CRServo.class,"frontServo");
    backServo = hardwareMap.get(CRServo.class,"backServo");

    motorShoot = hardwareMap.get(DcMotorEx.class, "motorShoot");
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger>0.05){
            frontServo.setPower(1);
        }else if(gamepad1.left_trigger>0.05){
            frontServo.setPower(-1);
        }else{
            frontServo.setPower(0);
        }
        if(gamepad2.right_trigger>0.05){
            backServo.setPower(1);
        }else if(gamepad2.left_trigger>0.05){
            backServo.setPower(-1);
        }else{
            backServo.setPower(0);
        }
        if(gamepad1.left_bumper){
            motorShoot.setPower(0.5);
        } else {
            motorShoot.setPower(0);
        }
    }

}

package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class encoderTesty extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Find a motor in the hardware map named "Arm Motor"
        DcMotor motor1 = hardwareMap.dcMotor.get("frontRight");
        DcMotor motor2 = hardwareMap.dcMotor.get("backRight");
        DcMotor motor3 = hardwareMap.dcMotor.get("backLeft");
        DcMotor motor4 = hardwareMap.dcMotor.get("frontLeft");


        // Reset the motor encoder so that it reads zero ticks
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while (opModeIsActive()) {
            // Get the current position of the motor
            int position1 = motor1.getCurrentPosition();
            int position2 = motor2.getCurrentPosition();
            int position3 = motor3.getCurrentPosition();
            int position4 = motor4.getCurrentPosition();

            // Show the position of the motor on telemetry
            telemetry.addData("Encoder Position fr", position1);
            telemetry.addData("Encoder Position br", position2);
            telemetry.addData("Encoder Position bl", position3);
            telemetry.addData("Encoder Position fr", position4);
            telemetry.update();
        }
    }
}
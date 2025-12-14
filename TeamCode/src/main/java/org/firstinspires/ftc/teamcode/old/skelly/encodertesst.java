package org.firstinspires.ftc.teamcode.old.skelly;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//@TeleOp
public class encodertesst extends OpMode {
    private DcMotor motorTurn;
    double leftBoundTicks = 0;
    double leftBoundDegrees = 0;
    double rightBoundTicks = 0;
    double rightBoundDegrees = 0;

    @Override
    public void init() {
        motorTurn = hardwareMap.get(DcMotor.class, "motorTurn");
        motorTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop() {
        double TICKS_PER_REV = 2150.8;
        double DEGREES_PER_TICK = 360/TICKS_PER_REV;

        int currentPos = motorTurn.getCurrentPosition();



        telemetry.addData("TicksPerRev", motorTurn.getCurrentPosition());
        telemetry.addData("Degrees", (currentPos * DEGREES_PER_TICK));

        if(gamepad1.dpadLeftWasPressed()){
            leftBoundTicks = currentPos;
            leftBoundDegrees = (currentPos * DEGREES_PER_TICK);
        }else if(gamepad1.dpadRightWasPressed()){
             rightBoundTicks = currentPos;
             rightBoundDegrees = (currentPos * DEGREES_PER_TICK);
        }

        telemetry.addData("RightBoundTicks", rightBoundTicks);
        telemetry.addData("LeftBoundTicks", leftBoundTicks);
        telemetry.addData("RightBoundDeg", rightBoundDegrees);
        telemetry.addData("LeftBoundDeg", leftBoundDegrees);
        motorTurn.setTargetPosition(350);
        motorTurn.setPower(0.1);
    }
}

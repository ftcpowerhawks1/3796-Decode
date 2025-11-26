package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.old.skelly.LimelightCode;
import org.firstinspires.ftc.teamcode.old.skelly.MecanumDriveCode;
import org.firstinspires.ftc.teamcode.old.skelly.ShooterCode;

import java.util.List;

@TeleOp
public class November extends OpMode {
    ShooterCode shoot = new ShooterCode();
    double shootSpeed = 0.00;
    private DcMotor intake, motorTurn;
    private Limelight3A limelight;
    MecanumDriveCode drive = new MecanumDriveCode();
    double forward, strafe, rotate;
    double speed = 1.0;
    LimelightCode limelightCode = new LimelightCode();
    //FOR TEST PURPOSES
    int LLmode = -1;

    @Override
    public void init() {
        drive.init(hardwareMap);
        shoot.init(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "motorIn");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelightCode.init(hardwareMap);
    }

    @Override
    public void start() {
        telemetry.addData("Pipeline", LLmode);
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
//DRIVE CODE
            //Establishes values that are used actively in drive
            forward = gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;
            //Uses variables to execute motor rotations
            drive.drive(forward, strafe, rotate, speed);
//TURRET TURN
            //Actively Tracks AprilTag as code runs

            if (llResult.isValid() && llResult != null) {
                double tx = llResult.getTx();
                if (tx > 2) {
                    motorTurn.setPower(0.5);
                } else if (tx < -2) {
                    motorTurn.setPower(-0.5);
                } else {
                    motorTurn.setPower(0);
                }

            }

            telemetry.addData("April Tag ID", limelightCode.getAprilTag());
            telemetry.addData("April Tag Distance", limelightCode.getAprilDistance());
//SHOOTER CODE
            //ShooterCode
            telemetry.addData("Shooter Speed", shootSpeed);
            if (gamepad2.xWasPressed()) {
                shootSpeed += 0.1;
            }
            if (gamepad2.bWasPressed()) {
                shootSpeed += -0.1;
            }
            if (gamepad2.right_trigger > 0.5) {
                shoot.setMotorSpeed(shootSpeed);
            } else {
                shoot.setMotorSpeed(0);
            }
            //TESTING
            //Switch Between Pipelines (TEST CODE REMOVE FOR COMP)
            if (gamepad1.dpadUpWasPressed()) {
                limelightCode.pipelineSwap(LLmode);
                LLmode++;
                if (LLmode > 2) {
                    LLmode = 2;
                } else {
                    LLmode += 1;
                }
                limelightCode.pipelineSwap(LLmode);
            } else if (gamepad1.dpadDownWasPressed()) {
                limelightCode.pipelineSwap(LLmode);
                if (LLmode < 0) {
                    LLmode = 0;

                } else {
                    LLmode -= 1;

                }
            }
//Intake Code
            if (gamepad1.right_trigger > 0.05) {
                intake.setPower(1);
            } else if (gamepad1.left_trigger > 0.05) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        }
    }

package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.old.skelly.LimelightCode;
import org.firstinspires.ftc.teamcode.skeleton.MecanumDriveCode;
import org.firstinspires.ftc.teamcode.old.skelly.ShooterCode;
import org.firstinspires.ftc.teamcode.skeleton.TurnTableMotor;
import org.firstinspires.ftc.teamcode.skeleton.shooter;

import java.util.List;

//@TeleOp
public class November2 extends OpMode {
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

           limelightCode.turnTable();

            telemetry.addData("April Tag ID", limelightCode.getAprilTag());
            telemetry.addData("April Tag Distance", limelightCode.getAprilDistance());
//SHOOTER CODE
            //ShooterCode
            telemetry.addData("Shooter Speed", shootSpeed);
            if (gamepad1.xWasPressed()) {
                shootSpeed += 0.1;
            }
            if (gamepad1.bWasPressed()) {
                shootSpeed += -0.1;
            }
            if (gamepad1.right_trigger > 0.5) {
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
            telemetry.addData("MOde", LLmode);

//Intake Code
            if (gamepad1.dpadLeftWasPressed()) {
                intake.setPower(1);
            } else if (gamepad1.dpadRightWasPressed()) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        }

    @TeleOp
    public static class November3 extends OpMode{
        TurnTableMotor turnTableMotor = new TurnTableMotor();
        MecanumDriveCode drive = new MecanumDriveCode();
        shooter shoot = new shooter();
        private DcMotor motorIntake;

        @Override
        public void init() {
            motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
            motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            drive.init(hardwareMap);
            shoot.init(hardwareMap);
            turnTableMotor.init(hardwareMap);
        }

        @Override
        public void loop() {
    //DRIVE
            double forward = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;
            double maxSpeed = 1.0;

            drive.drive(forward,strafe,rotate,maxSpeed);

    //TURRET
            turnTableMotor.track();


    //SHOOTER
            if(gamepad2.right_trigger > 0.05){
                shoot.shooterVelocity(1);
            }else{
                shoot.shooterVelocity(0);
            }

    //INTAKE
            if(gamepad1.left_trigger > 0.05){
                motorIntake.setPower(-1);
            }else if(gamepad1.right_trigger > 0.05){
                motorIntake.setPower(1);
            }else{
                motorIntake.setPower(0);
            }

        }

    }
}

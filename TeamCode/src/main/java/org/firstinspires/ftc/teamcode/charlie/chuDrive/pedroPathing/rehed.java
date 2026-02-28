package org.firstinspires.ftc.teamcode.charlie.chuDrive.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.charlie.chuDrive.packages.aimCode;
import org.firstinspires.ftc.teamcode.charlie.chuDrive.packages.mecanumDrive;
import org.firstinspires.ftc.teamcode.charlie.chuDrive.packages.shooterPIDF;
@Autonomous
public class rehed extends OpMode {
    double inPower = 1;
    aimCode turnTableMotor = new aimCode();
    mecanumDrive drive = new mecanumDrive();
    shooterPIDF shoot = new shooterPIDF();
    private CRServo intakeFront;     // Pulls artifacts in
    private CRServo intakeBack;
    // Flywheel shooter
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_POS_FINISH_POS,
        FINISHED_PATHS
    }
    PathState pathState;
    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(84, 85, Math.toRadians(50));
    private final Pose controlPoint1 = new Pose(69, 55);
    private PathChain driveStartPosShootPos, driveShootPoseStartPose;

    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, controlPoint1, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPoseStartPose = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPoint1, startPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), startPose.getHeading())
                .build();
    }
    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                pathState = PathState.SHOOT_POS_FINISH_POS;
                break;
            case SHOOT_POS_FINISH_POS:
                if(pathTimer.getElapsedTimeSeconds() < 10 && pathTimer.getElapsedTimeSeconds() > 2){
                    intakeFront.setPower(inPower);
                    intakeBack.setPower(-inPower);
                }
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 10) {
                    follower.followPath(driveShootPoseStartPose, true);
                    pathState = PathState.FINISHED_PATHS;
                }
                break;
            case FINISHED_PATHS:
                if(!follower.isBusy()){
                    telemetry.addLine("Finished Paths");
                }
                break;
            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }
    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        telemetry.addData("InPower", inPower);
        intakeFront = hardwareMap.get(CRServo.class, "frontServo");
        intakeBack = hardwareMap.get(CRServo.class, "backServo");
        drive.init(hardwareMap);
        //MODE = 1 BLUE || MODE = 2 RED
        shoot.init(hardwareMap,2);
        turnTableMotor.init(hardwareMap);
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));
        buildPaths();
        follower.setPose(startPose);
    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        turnTableMotor.track();
        shoot.ShooterVelocity(1);
        follower.update();
        if(!follower.isBusy()){
            statePathUpdate();
        }

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.firstComp.shooterCOMP;
import org.firstinspires.ftc.teamcode.skeleton.TurnTableMotor;

@Autonomous
public class tryAgainAgain extends OpMode {
    private Limelight3A limelight;
    private DcMotor motorIntake;
    shooterCOMP shoot = new shooterCOMP();
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }
    PathState pathState;
    private final Pose startPose = new Pose(60, 9, Math.toRadians(90));
    private final Pose shootPose = new Pose(60, 35, Math.toRadians(180));
    private PathChain driveStartPosShootPos;

    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }
    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path");
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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60, 9, Math.toRadians(90)));
        buildPaths();
        follower.setPose(startPose);
    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
        limelight.start();
    }

    @Override
    public void loop() {

        follower.update();
        statePathUpdate();
        shoot.ShooterVelocity(1);
        telemetry.addData("Distance (cm)",shoot.distance());
        telemetry.addData("Power Target", shoot.getShootValue());
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        if(pathTimer.getElapsedTimeSeconds() > 10 && pathTimer.getElapsedTimeSeconds() < 16){
            motorIntake.setPower(1);
        } else if (pathTimer.getElapsedTimeSeconds() > 19 && pathTimer.getElapsedTimeSeconds() < 21) {
            motorIntake.setPower(1);
        } else if (pathTimer.getElapsedTimeSeconds() > 24 && pathTimer.getElapsedTimeSeconds() < 27) {
            motorIntake.setPower(1);
        } else {
            motorIntake.setPower(0);
        }
    }
}

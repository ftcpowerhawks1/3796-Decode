package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.skeleton.TurnTableMotor;
import org.firstinspires.ftc.teamcode.skeleton.shooter2;
import org.firstinspires.ftc.teamcode.pedroPathing.motors.motorIn;

@Autonomous
public class tryAgain extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private TurnTableMotor turnTableMotor;
    private shooter2 shoot2;

    public enum PathState{
        DRIVE_STARTPOS_SHOOT_POS,
        DRIVE_SHOOT_POS_OTHER_POS,
        DRIVE_OTHER_POS_START_POS,
        SHOOT_PRELOAD
    }
    PathState pathState;
    private final Pose startPose = new Pose(60, 9, Math.toRadians(90));
    private final Pose shootPose = new Pose(35, 35, Math.toRadians(180));
    private final Pose otherPose = new Pose(85, 35, Math.toRadians(0));
    private PathChain driveStartPosShootPos, driveShootPosOtherPos, driveOtherPoseStartPose;

    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosOtherPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, otherPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), otherPose.getHeading())
                .build();
        driveOtherPoseStartPose = follower.pathBuilder()
                .addPath(new BezierLine(otherPose, startPose))
                .setLinearHeadingInterpolation(otherPose.getHeading(), startPose.getHeading())
                .build();
    }
    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                pathState = PathState.DRIVE_SHOOT_POS_OTHER_POS;
                break;
            case DRIVE_SHOOT_POS_OTHER_POS:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 10) {
                    telemetry.addLine("idk what to put here");
                    follower.followPath(driveShootPosOtherPos, true);
                    pathState = PathState.DRIVE_OTHER_POS_START_POS;
                }
                break;
            case DRIVE_OTHER_POS_START_POS:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 20) {
                    telemetry.addLine("idk what to put here");
                    follower.followPath(driveOtherPoseStartPose, true);
                    pathState = PathState.SHOOT_PRELOAD;
                }
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
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}

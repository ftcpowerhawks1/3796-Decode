package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@Autonomous
public class SimpleAutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public enum PathState {
        //START POSITION_END POSITION
        // Drive > Movement State
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }
    PathState pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose intakePose = new Pose(56, 36, Math.toRadians(180));
    private PathChain driveStartPosIntakePos;
    public void buildPaths(){
        driveStartPosIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, intakePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), intakePose.getHeading())
                .build();
    }
    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosIntakePos, true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Path Done");
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
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
        //Init mechanisms n shi
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

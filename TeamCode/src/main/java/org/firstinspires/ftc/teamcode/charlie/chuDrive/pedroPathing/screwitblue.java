package org.firstinspires.ftc.teamcode.charlie.chuDrive.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.charlie.chuDrive.packages.aimCode;
import org.firstinspires.ftc.teamcode.charlie.chuDrive.packages.mecanumDrive;
import org.firstinspires.ftc.teamcode.charlie.chuDrive.packages.shooterPIDF;
import org.opencv.core.Mat;
@Autonomous
public class screwitblue extends OpMode {
    aimCode turnTableMotor = new aimCode();
    mecanumDrive drive = new mecanumDrive();
    shooterPIDF shoot = new shooterPIDF();
    private CRServo intakeFront;
    private CRServo intakeBack;


    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        starttoshoot,
        shoottoreadycollect1,
        collect1,
        collect1toshoot,
        shoottoreadycollect2,
        collect2,
        collect2toshoot,
        shoottoreadycollect3,
        collect3,
        collect3toshoot,
        FINISHED_PATHS
    }
    PathState pathState;
    public PathChain Starttoshoot;
    public PathChain shoottoreadycollect1;
    public PathChain Collect1;
    public PathChain Coollect1toshoot;
    public PathChain shoottoreadycollect2;
    public PathChain collect2;
    public PathChain collect2toshoot;
    public PathChain shoottocollect3;
    public PathChain collect3;
    public PathChain collect3toshoot;
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(60, 85, Math.toRadians(135));
    private final Pose readyToCollect1 = new Pose(42, 84, Math.toRadians(180));
    private final Pose Collect1Pose = new Pose(20, 84, Math.toRadians(180));
    private final Pose readyToCollect2 = new Pose(42, 60, Math.toRadians(180));
    private final Pose Collect2Pose = new Pose(20, 60, Math.toRadians(180));
    private final Pose readyToCollect3 = new Pose(42, 36, Math.toRadians(180));
    private final Pose Collect3Pose = new Pose(20, 36, Math.toRadians(180));


    private final Pose controlPoint1 = new Pose(75, 55);

    public void buildPaths(){
        Starttoshoot = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, controlPoint1, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shoottoreadycollect1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, readyToCollect1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readyToCollect1.getHeading())
                .build();
        Collect1 = follower.pathBuilder()
                .addPath(new BezierLine(readyToCollect1, Collect1Pose))
                .setLinearHeadingInterpolation(readyToCollect1.getHeading(), Collect1Pose.getHeading())
                .build();
        Coollect1toshoot = follower.pathBuilder()
                .addPath(new BezierLine(Collect1Pose, shootPose))
                .setLinearHeadingInterpolation(Collect1Pose.getHeading(), shootPose.getHeading())
                .build();
        shoottoreadycollect2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, readyToCollect2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Collect2Pose.getHeading())
                .build();
        collect2 = follower.pathBuilder()
                .addPath(new BezierLine(readyToCollect2, Collect2Pose))
                .setLinearHeadingInterpolation(readyToCollect2.getHeading(), Collect2Pose.getHeading())
                .build();
        collect2toshoot = follower.pathBuilder()
                .addPath(new BezierLine(Collect2Pose, shootPose))
                .setLinearHeadingInterpolation(Collect2Pose.getHeading(), shootPose.getHeading())
                .build();
        shoottocollect3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, readyToCollect3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readyToCollect3.getHeading())
                .build();
        collect3 = follower.pathBuilder()
                .addPath(new BezierLine(readyToCollect3, Collect3Pose))
                .setLinearHeadingInterpolation(readyToCollect3.getHeading(), Collect3Pose.getHeading())
                .build();
        collect3toshoot = follower.pathBuilder()
                .addPath(new BezierLine(Collect3Pose, shootPose))
                .setLinearHeadingInterpolation(Collect3Pose.getHeading(), shootPose.getHeading())
                .build();
    }
    public void statePathUpdate(){
        switch (pathState){
            case starttoshoot:
                follower.followPath(Starttoshoot, true);
                pathState = PathState.shoottoreadycollect1;
                break;
            case shoottoreadycollect1:
                if(pathTimer.getElapsedTimeSeconds() < 10 && pathTimer.getElapsedTimeSeconds() > 2){
                    intakeFront.setPower(1);
                    intakeBack.setPower(-1);
                }
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 10) {
                    follower.followPath(shoottoreadycollect1, true);
                    pathState = screwitblue.PathState.FINISHED_PATHS;
                }
                break;
            case FINISHED_PATHS:
                if(!follower.isBusy()){
                    telemetry.addLine("Finished Paths");
                    turnTableMotor.resetAim();
                }
                break;
            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }
    public void setPathState(screwitblue.PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        telemetry.addData("InPower", 1);
        intakeFront = hardwareMap.get(CRServo.class, "frontServo");
        intakeBack = hardwareMap.get(CRServo.class, "backServo");
        drive.init(hardwareMap);
        //MODE = 1 BLUE || MODE = 2 RED
        shoot.init(hardwareMap,1);
        turnTableMotor.init(hardwareMap);
        pathState = PathState.starttoshoot;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        turnTableMotor.track();
        shoot.ShooterVelocity(1);
        follower.update();
        if(!follower.isBusy()){
            statePathUpdate();
        }
    }
}

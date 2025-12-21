package org.firstinspires.ftc.teamcode.skeleton;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class updatePosition {
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    private void init(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");

    }

    private void locate(){
        LLResult llResult = limelight.getLatestResult();
        if(llResult.isValid()){
            Pose3D botpose = llResult.getBotpose_MT2();
        }
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,llResult.getBotpose().getPosition().x , llResult.getBotpose().getPosition().y, AngleUnit.RADIANS, 0));

        pinpoint.setOffsets(-2.625, -6.625, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();

    }
}

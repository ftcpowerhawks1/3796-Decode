package org.firstinspires.ftc.teamcode.firstComp;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.skeleton.TurnTableMotor;

public class limelightAuto {
    shooterCOMP shoot = new shooterCOMP();
    TurnTableMotor turnTable = new TurnTableMotor();
    private DcMotor motorIntake;

    /* PUT ALL OF THIS IN THE INIT
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        turnTableMotor.init(hardwareMap);

        MODE = 1 BLUE || MODE = 2 RED
        CHANGE AS NEEDED vvv
        shoot.init(hardwareMap,1);

    */

    /* PUT THIS IN YOUR LOOP

    turnTableMotor.track();

    Call this if you want distance
    shoot.distance();

     */
}

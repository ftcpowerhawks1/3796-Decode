package org.firstinspires.ftc.teamcode.old.skelly;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp
public class LIMELIGHTTEST extends OpMode {

    LimelightCode limelightCode = new LimelightCode();
    @Override
    public void init() {
        limelightCode.init(hardwareMap);
    }

    @Override
    public void loop() {
        limelightCode.turnTable();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(group = "drive")
public class redNearSimplyPark extends AutoCommon {
    @Override
    public void runOpMode() {
        super.runOpMode();

        driveOnHeading(-48, 0.3, 0);
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(group = "drive")
public class nearSideParkAndStrafeOnly extends AutoCommon {
    @Override
    public void runOpMode() {
        super.runOpMode();

        strafeOnHeading(24,0.3,0);
    }
}

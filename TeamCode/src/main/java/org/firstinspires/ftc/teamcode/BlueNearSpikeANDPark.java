package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "drive")
public class BlueNearSpikeANDPark extends AutoCommon {
    @Override
    public void runOpMode() {
        super.runOpMode();

        driveOnHeading(60, 0.3, 0);
        driveOnHeading(-12,0.3,0);
        spikeDump();
        driveOnHeading(-20,0.5,0);
        turnToHeading(90,0.3);
        driveOnHeading(45,0.5,90);
        strafeOnHeading(-10,0.3,90);
        sleep(1000);
        intakeOn(1000);
        sleep(2000);
        spikeDump();
    }
}
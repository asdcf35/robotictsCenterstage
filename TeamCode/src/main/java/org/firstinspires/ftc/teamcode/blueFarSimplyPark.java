package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "drive")
public class blueFarSimplyPark extends AutoCommon {
    @Override
    public void runOpMode() {
        telemetry.addLine("This is the Blue Alliance FarSide Simply Parking.");
        telemetry.addLine("Please put the robot with the front side facing the Backdrop. Robot should be placed in the row with the spike marks.");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            driveOnHeading(-24, 0.3, 0);
            sleep(200);
            driveOnHeading(-24 * 2, 0.3, 0);
            sleep(200);
            turnToHeading(-90, 0.3);
            sleep(200);
            driveOnHeading(-24, 0.3, 0);
            sleep(200);
            turnToHeading(90, 0.3);
            telemetry.addLine("Auto Finished. Parked in correct location");
            telemetry.update();
        }
    }
}

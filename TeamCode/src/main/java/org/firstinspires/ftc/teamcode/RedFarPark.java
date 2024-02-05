package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "drive")
public class RedFarPark extends AutoCommon {
    @Override
    public void runOpMode() {
        super.runOpMode();

        driveOnHeading(100, 0.3, 0);
        driveOnHeading(-8,0.3,0);
        robot.lift.servoIntake.setPosition(robot.lift.SERVO_INTAKE_REG_POS);
        turnToHeading(-90,0.3);
        driveOnHeading(130,0.5,-90);
        strafeOnHeading(15,0.3,-90);
        sleep(1000);
        spikeDump();
    }
}
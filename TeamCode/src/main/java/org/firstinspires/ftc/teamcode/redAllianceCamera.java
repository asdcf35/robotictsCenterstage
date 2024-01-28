package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "drive")
public class redAllianceCamera extends AutoCommon {
    @Override
    public void runOpMode() {
        super.runOpMode();
        waitForStart();
//        driveOnHeading(10,1,0);
        if(currentSpikePos == SpikePosition.CENTER){
            telemetry.addData("Location(got from redAllianceCamera): ", "Center");
        }
        else if(currentSpikePos == SpikePosition.LEFT){
            telemetry.addData("Location(got from redAllianceCamera): ", "Left");
        }
        else if(currentSpikePos == SpikePosition.RIGHT){
            telemetry.addData("Location(got from redAllianceCamera): ", "Right");
        }
        else{
            telemetry.addLine("RedAllianceCamera opmode doesn't get any spike position from robot hardware");
        }
        telemetry.update();
    }

    //positive -> right
    //negative -> left
//    private void middle(){
//        telemetry.addLine("Middle Spike Mark Autonomous");
//        driveOnHeading(46,0.3,0);
//        spikeDump();
////        robot.turnOnIntake();
////        driveOnHeading(40,0.5,0);
////        turnToHeading(90, 0.5);
////        driveOnHeading(72,0.3,90);
////        strafeOnHeading(-35, 0.5,90);
////        liftPos(robot.lift.LIFT_LOW_POS);
////        driveOnHeading(15,0.3,90);
////        sleep(1000);
////        dump();
////        liftPos(robot.lift.LIFT_INTAKE_POS);
////        strafeOnHeading(50,1,90);
////        driveOnHeading(10,0.5,90);
//
//    }
}

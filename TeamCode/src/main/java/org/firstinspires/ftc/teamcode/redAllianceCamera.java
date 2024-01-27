package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "drive")
public class redAllianceCamera extends AutoCommon {
    @Override
    public void runOpMode() {
        super.runOpMode();
        waitForStart();
//        driveOnHeading(10,1,0);
        if(pos == 2) {
            telemetry.addData("Location: ", "Middle");
            driveOnHeading(46,0.3,0);
            spikeDump();
            sleep(100000);
//            middle(); //middle spike mark
        } else if (pos == 1) {
            telemetry.addData("Location", "left");
            sleep(100000);
//            left();//left spike mark
        } else {
            telemetry.addData("Location:", "Right");
            sleep(100000);
//            right();//right spike mark
            turnToHeading(90,0.2);
//            driveOnHeading(5,1,90);
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

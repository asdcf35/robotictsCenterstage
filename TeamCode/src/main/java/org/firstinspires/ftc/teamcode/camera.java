package org.firstinspires.ftc.teamcode;

public class camera extends AutoCommon {
    @Override
    public void runOpMode() {
        super.runOpMode();

    }
    private void middle(){
        driveOnHeading(46,0.3,0);
        //dump
        robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_OUTAKE_POS);
        driveOnHeading(40,0.5,0);
        turnToHeading(90, 0.5);
        driveOnHeading(72,0.3,90);
        strafeOnHeading(-35, 0.5,90);
        //lift
        robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_INIT_POS);
        robot.Medium();
        driveOnHeading(15,0.3,90);
        //dump
        robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_OUTAKE_POS);
        sleep(1000);
        //retract
        robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_INIT_POS);
        robot.Medium();
        sleep(1000);
        strafeOnHeading(50,1,90);
        driveOnHeading(10,0.5,90);

    }
}

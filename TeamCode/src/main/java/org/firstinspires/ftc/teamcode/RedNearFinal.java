package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class RedNearFinal extends AutoCommon {

    YellowBlobDetectionPipeline yellowBlobDetectionPipeline;
    OpenCvCamera camera;
    @Override
    public void runOpMode()
    {
        super.runOpMode();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(12,-61.5,Math.toRadians(90));

        drive.setPoseEstimate(startPose);


        if (pos == 2) {
            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(12,-20),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj1);

            Pose2d pose1 = new Pose2d(12,-20,Math.toRadians(90));
            Trajectory traj2 = drive.trajectoryBuilder(pose1)
                    .lineToConstantHeading(new Vector2d(12,-30),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj2);

            spikeDump();

            Pose2d pose2 = new Pose2d(12,-30,Math.toRadians(90));
            Trajectory traj3 = drive.trajectoryBuilder(pose2)
                    .lineToConstantHeading(new Vector2d(12,-35),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj3);

//            robot.lift.servoIntake.setPosition(robot.lift.LIFT_INTAKE_POS);
            turnToHeading(-85,0.3);
            intakeOn(1500);
            liftPos(1700);

            Pose2d pose3 = new Pose2d(12,-35,Math.toRadians(0));
            Trajectory traj4 = drive.trajectoryBuilder(pose3)
                    .lineToConstantHeading(new Vector2d(52.5,-35),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj4);

            Pose2d pose4 = new Pose2d(52.5,-35,Math.toRadians(0));
            Trajectory traj5 = drive.trajectoryBuilder(pose4)
                    .strafeLeft(10)
                    .build();
            drive.followTrajectory(traj5);

            robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_OUTAKE_POS);
            sleep(500);


            Pose2d pose5 = new Pose2d(52.5,-25,Math.toRadians(0));
            Trajectory traj6 = drive.trajectoryBuilder(pose5)
                    .lineToConstantHeading(new Vector2d(50,-25),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj6);
            sleep(500);
            robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_INTAKE_POS);

            liftPos(0);
            Pose2d pose6 = new Pose2d(52.5,-25,Math.toRadians(0));
            Trajectory traj7 = drive.trajectoryBuilder(pose6)
                    .strafeRight(35)
                    .build();
            drive.followTrajectory(traj7);
            spikeDump();




        }

        else if (pos == 3){
            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(30,-37.5),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj1);
            spikeDump();

            Pose2d pose1 = new Pose2d(30,-37.5,Math.toRadians(90));
            Trajectory traj2 = drive.trajectoryBuilder(pose1)
                    .lineToConstantHeading(new Vector2d(30,-50),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj2);

            turnToHeading(-85,0.3);
            intakeOn(1500);
            liftPos(1700);

            Pose2d pose2 = new Pose2d(30,-37.5,Math.toRadians(0));
            Trajectory traj3 = drive.trajectoryBuilder(pose2)
                    .lineToConstantHeading(new Vector2d(54,-37.5),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj3);

            Pose2d pose3 = new Pose2d(54,-37.5,Math.toRadians(0));
            Trajectory traj4 = drive.trajectoryBuilder(pose3)
                    .strafeLeft(19.5)
                    .build();
            drive.followTrajectory(traj4);

            robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_OUTAKE_POS);
            sleep(500);

            Pose2d pose4 = new Pose2d(54,-25,Math.toRadians(0));
            Trajectory traj5 = drive.trajectoryBuilder(pose4)
                    .lineToConstantHeading(new Vector2d(52.5,-18),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj5);

            sleep(500);
            robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_INTAKE_POS);
            liftPos(0);
            Pose2d pose5 = new Pose2d(52.5,-18,Math.toRadians(0));
            Trajectory traj6 = drive.trajectoryBuilder(pose5)
                    .strafeRight(30)
                    .build();
            drive.followTrajectory(traj6);



            spikeDump();


        }


        else if (pos == 1){
            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(12,-25.5, Math.toRadians(180)))
                    .build();
            drive.followTrajectory(traj1);

            Pose2d pose1 = new Pose2d(12,-25.5,Math.toRadians(180));
            Trajectory traj2 = drive.trajectoryBuilder(pose1)
                    .lineToConstantHeading(new Vector2d(11,-25.5),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj2);
            spikeDump();

            Pose2d pose2 = new Pose2d(11,-25.5,Math.toRadians(180));
            Trajectory traj3 = drive.trajectoryBuilder(pose2)
                    .lineToConstantHeading(new Vector2d(54,-25.5),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj3);

            intakeOn(1500);
            sleep(1500);
            turnToHeading(-85,0.3);
            liftPos(1700);

            Pose2d pose3 = new Pose2d(54,-25.5,Math.toRadians(0));
            Trajectory traj4 = drive.trajectoryBuilder(pose3)
                    .strafeLeft(7)
                    .build();
            drive.followTrajectory(traj4);

            robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_OUTAKE_POS);
            sleep(500);

            Pose2d pose4 = new Pose2d(54,-18.5,Math.toRadians(0));
            Trajectory traj5 = drive.trajectoryBuilder(pose4)
                    .lineToConstantHeading(new Vector2d(52.5,-18.5),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(traj5);
            sleep(500);

            robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_INIT_POS);
            liftPos(0);

            Pose2d pose5 = new Pose2d(52.5,-18.5,Math.toRadians(0));
            Trajectory traj6 = drive.trajectoryBuilder(pose5)
                    .strafeRight(40)
                    .build();
            drive.followTrajectory(traj6);

//            spikeDump();

        }


    }
}
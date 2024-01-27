package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class robotTeleOp extends OpMode {
    RobotHardware robot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap, true, true, true, true);       // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void start() {
        telemetry.addData("Status", "Running");


    }

    @Override
    public void loop() {
        driveControl();
        liftControl();
        hangControl();
        manualControl();
        bucketControl();
        launchControl();
        intakeControl();
//        airplaneLaunch();
    }

    private void driveControl() {
        double scale = 0.4;
        if (gamepad1.left_bumper) {
            gamepad1.rumble(500);
            scale = 0.7;
        } else if (gamepad1.left_trigger > 0.5) {
            scale = 0.3;
        }
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        robot.driveTrain.startMove(drive, strafe, turn, scale);
        robot.telemetryUpdate(telemetry);

    }

    private void liftControl() {
        if (gamepad1.a) {
            robot.Intake();
        }
        if (gamepad1.b) {
            robot.Hover();
        }
        if (gamepad1.x) {
            robot.Low();
        }
        if (gamepad1.y) {
            robot.Medium();
        }

//        if(robot.currentState != RobotHardware.States.INTAKE){
//            robot.lift.motorIntakeHighSpeed.setPower(0);
//            robot.lift.servoIntake.setPosition(robot.lift.SERVO_INTAKE_HOVER_POS);
//        }
    }

    private void bucketControl() {
        if ( gamepad1.right_trigger > 0.3 && robot.lift.motorLift.getCurrentPosition() > 50) {
            robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_OUTAKE_POS);
        } else {
            robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_INIT_POS);
        }
    }

    private void launchControl() {
        if (gamepad1.dpad_right) {
            robot.lift.servoLauncher.setPosition(robot.lift.SERVO_LAUNCHER_LAUNCH_POS);
        }
    }

    private void manualControl() {
        if (gamepad1.right_bumper) {
            robot.lift.servoIntake.setPosition(robot.lift.SERVO_INTAKE_JAM_POS);
        } else {
            robot.lift.servoIntake.setPosition(robot.lift.SERVO_INTAKE_REG_POS);
        }
//        if(gamepad1.dpad_left) {
//            robot.lift.motorIntakeHighSpeed.setPower(-robot.lift.INTAKE_HS_POWER);
//            robot.lift.motorIntakeLowSpeed.setPower(-robot.lift.INTAKE_LS_POWER);
//        }
    }
    private void hangControl(){
    }

    private void intakeControl() {
        if (gamepad1.a) {
            robot.lift.motorIntakeHighSpeed.setPower(robot.lift.INTAKE_HS_POWER);
            robot.lift.motorIntakeLowSpeed.setPower(robot.lift.INTAKE_LS_POWER);
        } else if (gamepad1.b) {
            robot.lift.motorIntakeHighSpeed.setPower(0);
            robot.lift.motorIntakeLowSpeed.setPower(0);
        } else if (gamepad1.dpad_left) {
            robot.lift.motorIntakeHighSpeed.setPower(-robot.lift.INTAKE_HS_POWER);
            robot.lift.motorIntakeLowSpeed.setPower(-robot.lift.INTAKE_LS_POWER);
        }
    }

//    private void airplaneLaunch() {
//        if (gamepad1.dpad_right) {
//            robot.airplane.launch();
//        }
    //}
}

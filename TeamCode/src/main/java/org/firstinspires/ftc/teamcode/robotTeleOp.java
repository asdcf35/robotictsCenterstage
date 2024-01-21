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

        robot = new RobotHardware(hardwareMap, false, false, true, false);       // Tell the driver that initialization is complete.
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
//        airplaneLaunch();
    }

    private void driveControl() {
        double scale = 0.3;
        if (gamepad1.left_bumper) {
            gamepad1.rumble(500);
            scale = 0.8;
        } else if (gamepad1.left_trigger > 0.5) {
            scale = 0.1;
        }
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        robot.driveTrain.startMove(drive, strafe, turn, scale);
        robot.telemetryUpdate(telemetry);

    }

    private void liftControl() {
        if (gamepad1.a) {
            robot.Intake();
        }
        if (gamepad1.b) {
            robot.Low();
        }
        if (gamepad1.x) {
            robot.Medium();
        }
        if (gamepad1.y) {
            robot.High();
        }
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            robot.Hover();
        }
        if(robot.currentState != RobotHardware.States.INTAKE){
            robot.lift.motorIntakeHighSpeed.setPower(0);
            robot.lift.servoIntake.setPosition(robot.lift.SERVO_INTAKE_HOVER_POS);
        }
    }

    private void manualControl() {
        if (gamepad1.dpad_left) {
            robot.lift.servoIntake.setPosition(0);
        }
        if (gamepad1.dpad_right) {
            robot.lift.servoIntake.setPosition(0.25);
        }
        if (gamepad1.dpad_up) {
            robot.lift.servoIntake.setPosition(0.5);
        }
        if (gamepad1.dpad_down)
        {
            robot.lift.servoIntake.setPosition(0.75);
        }
    }
    private void hangControl(){
    }
//    private void airplaneLaunch() {
//        if (gamepad1.dpad_right) {
//            robot.airplane.launch();
//        }
    //}
}

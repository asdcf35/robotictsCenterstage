package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class robotTeleOp extends OpMode {
    RobotHardware robot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap);

        // Tell the driver that initialization is complete.
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
        manualControl();
        airplaneLaunch();
    }

    private void driveControl() {
        double scale = 0.8;
        if (gamepad1.left_bumper) {
            gamepad1.rumble(500);
            scale = 1;
        } else if (gamepad1.left_trigger > 0.5) {
            scale = 0.5;
        }

        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        robot.startMove(drive, strafe, turn, scale);

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
    }

    private void manualControl() {

    }

    private void airplaneLaunch() {
        if (gamepad1.dpad_right) {
            robot.launch();
        }
    }
}

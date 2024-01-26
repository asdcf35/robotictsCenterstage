package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Intake Test", group = "TeleOp")
public class testCenterstage extends LinearOpMode {
    private RobotHardware robot;
    @Override
    public void runOpMode() {
        // Initialize motors test
        robot = new RobotHardware(hardwareMap,false,false,true,false);

        // Wait for the start button to be pressed
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                robot.lift.servoIntake.setPosition(0.28);
            } else {
                robot.lift.servoIntake.setPosition(robot.lift.SERVO_INTAKE_REG_POS);
            }

            if (gamepad1.a) {
                robot.lift.motorIntakeHighSpeed.setPower(0.7);
                robot.lift.motorIntakeLowSpeed.setPower(0.7);

            } else {
                robot.lift.motorIntakeHighSpeed.setPower(0);
                robot.lift.motorIntakeLowSpeed.setPower(0);
            }

            if (gamepad1.dpad_down) {
                robot.lift.servoBucket.setPosition(0.15);
            }
            if (gamepad1.dpad_up) {
                robot.lift.servoBucket.setPosition(0.25);
            }
            if (gamepad1.dpad_right) {
                robot.lift.servoBucket.setPosition(0.47);
            }
            if (gamepad1.dpad_left) {
                robot.lift.servoBucket.setPosition(0.75);
            }

            if (gamepad1.right_trigger > 0.5) {
                robot.lift.servoBucket.setPosition(0.15);
            } else {
                robot.lift.servoBucket.setPosition(0.47);
            }

            // to improve scale each sensitivity on trigger scale 0.1-1 of right trigger to position of servo .

        }

    }
}
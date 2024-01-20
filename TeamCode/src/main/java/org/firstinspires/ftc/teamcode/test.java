package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake Test", group = "TeleOp")
public class test extends LinearOpMode {
    private RobotHardware robot;
    @Override
    public void runOpMode() {
        // Initialize motors test
        robot = new RobotHardware(hardwareMap,false,false,true,false);

        // Wait for the start button to be pressed
        waitForStart();
        while (opModeIsActive()) {
            // Gamepad input for driv
            robot.Medium();
        }
    }
}


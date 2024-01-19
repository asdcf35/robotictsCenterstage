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

    private DcMotor rightSlides;
    private DcMotor leftSlides;
    private DcMotor intake;
    private Servo intakeServo;
    @Override
    public void runOpMode() {
        // Initialize motors test
        intake = hardwareMap.dcMotor.get("intake");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        // Reverse the direction of one motor on each side (if necessary)
        intake.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the start button to be pressed
        waitForStart();
        while (opModeIsActive()) {
            // Gamepad input for driving
            intake.setPower(0.9);
//            intakeServo.setPosition(0.3);
            telemetry.addData("Release Servo Target", intakeServo.getPosition());
            telemetry.update();
        }
    }
}


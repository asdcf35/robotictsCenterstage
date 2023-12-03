package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Aux Motor Test", group = "TeleOp")
public class test extends LinearOpMode {

    private DcMotor rightSlides;
    private DcMotor leftSlides;
    private DcMotor intake;
    @Override
    public void runOpMode() {
        // Initialize motors test
        rightSlides = hardwareMap.dcMotor.get("rightSlides");
        leftSlides = hardwareMap.dcMotor.get("leftSlides");
        intake = hardwareMap.dcMotor.get("intake");

        // Reverse the direction of one motor on each side (if necessary)
        leftSlides.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the start button to be pressed
        waitForStart();
        while (opModeIsActive()) {
            // Gamepad input for driving
            intake.setPower(0.9);
            sleep(500);
            leftSlides.setPower(0);
            rightSlides.setPower(0);
        }
    }
}


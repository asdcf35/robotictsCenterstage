package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class slidesTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int armUpPosition = 1000;
        int armDownPosition = 1000;

        DcMotor leftSlides = hardwareMap.dcMotor.get("leftSlides");
        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setTargetPosition(armDownPosition);
        leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DcMotor rightSlides = hardwareMap.dcMotor.get("rightSlides");
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setTargetPosition(armDownPosition);
        rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");

        waitForStart();

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {
            if (gamepad1.a) {
                leftSlides.setTargetPosition(armUpPosition);
                leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlides.setPower(0.5);

                rightSlides.setTargetPosition(armUpPosition);
                rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlides.setPower(0.5);
            }
            if (gamepad1.b) {
                leftSlides.setTargetPosition(armDownPosition);
                leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlides.setPower(0.5);

                rightSlides.setTargetPosition(armDownPosition);
                rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlides.setPower(0.5);
            }
            double position = leftSlides.getCurrentPosition();
            double desiredPosition = leftSlides.getTargetPosition();
            telemetry.addData("Encoder Position", position);
            telemetry.addData("Desired Position", desiredPosition);
            telemetry.update();

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            intake.setPower(-0.9);
        }
    }
}

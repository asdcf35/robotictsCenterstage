package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Basic RC OpMode", group = "TeleOp")
public class BasicRCOpMode extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        // Reverse the direction of one motor on each side (if necessary)
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Gamepad input for driving
            double drivePower = gamepad1.left_stick_y;
            double rotatePower = gamepad1.right_stick_x;
            double lrStrafeInput = -gamepad1.left_stick_x;
            // Set motor power
            leftFront.setPower(drivePower - rotatePower);
            rightFront.setPower(drivePower + rotatePower);
            leftRear.setPower(drivePower - rotatePower);
            rightRear.setPower(drivePower + rotatePower);

            if(lrStrafeInput==1){
                leftFront.setPower(1);
                rightFront.setPower(-1);
                leftRear.setPower(1);
                rightRear.setPower(-1);
            }


        }
    }
}


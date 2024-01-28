package org.firstinspires.ftc.teamcode;


import android.graphics.Bitmap;
import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.newDir.SkystoneDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoCommon extends LinearOpMode {
    protected RobotHardware robot;
    public void runOpMode() {

        robot = new RobotHardware(hardwareMap, true, true, true, true);
        initialHeading = getHeading();
        telemetry.update();

        telemetry.update();

        telemetry.addData("Status", "Initialized...");
        telemetry.update();

        waitForStart();

        // Release resources
    }

    private double inchesToTicks(double inches) {
        return inches * robot.DRIVE_MOTOR_TICKS_PER_ROTATION / (robot.WHEEL_DIAMETER * Math.PI);
    }

    private double initialHeading = 0;

    protected double getHeading() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - initialHeading;
    }

    protected double getHeadingDiff(double targetHeading) {
        double headingDiff = getHeading() - targetHeading;
        while (headingDiff > 180) {
            headingDiff -= 360;
        }
        while (headingDiff < -180) {
            headingDiff += 360;
        }
        return headingDiff;
    }

    protected void drive(double distance, double power) {
        double dir = Math.signum(distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(1, 0, 0, Math.abs(power) * dir);
        while (opModeIsActive() && Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) > encoderTicks);
        robot.driveTrain.stopMove();
    }

    protected void driveOnHeading(double distance, double power, double targetHeading) {
//        targetHeading = -targetHeading;
//        distance = -distance;
        double dir = Math.signum(distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(1, 0, 0, Math.abs(power) * dir);
        while (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks && opModeIsActive()) {
            telemetry.addData("current heading", getHeading());
            double turnMod = getHeadingDiff(targetHeading) / 100;
            telemetry.addData("turn mod", turnMod);
            robot.driveTrain.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
            telemetry.update();
        }
        robot.driveTrain.stopMove();
    }

    protected void driveOnHeadingRamp(double driveDistance, double minPower, double maxPower, double rampDistance, double targetHeading) {
        double dir = Math.signum(driveDistance);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(driveDistance));
        double rampTicks = inchesToTicks(Math.abs(rampDistance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(1, 0, 0, Math.abs(minPower) * dir);
        while (opModeIsActive() && Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks) {
            double turnMod = getHeadingDiff(targetHeading) / 100;
            double startRampPower = minPower + (maxPower - minPower) * (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) / rampTicks);
            double endRampPower = minPower + (maxPower - minPower) * (Math.abs(encoderTicks - robot.driveTrain.motorFL.getCurrentPosition()) / (rampTicks * 2));
            double power = Range.clip(Math.min(startRampPower, endRampPower), minPower, maxPower);
            robot.driveTrain.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
        }
        robot.driveTrain.stopMove();
    }


    protected void driveOnHeadingFlipped(double distance, double power, double targetHeading) {
        double dir = Math.signum(distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(1, 0, 0, Math.abs(power) * dir);
        while (opModeIsActive() && Math.abs(robot.driveTrain.motorFR.getCurrentPosition()) < encoderTicks) {
            double turnMod = getHeadingDiff(targetHeading) / 100;
            robot.driveTrain.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
        }
        robot.driveTrain.stopMove();
    }

    protected void strafeOnHeading(double distance, double power, double targetHeading) {
        double dir = Math.signum(distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(0, 1, 0, Math.abs(power) * dir);
        while (opModeIsActive() && Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks) {
            double turnMod = getHeadingDiff(targetHeading) / 100;
            robot.driveTrain.startMove(0, Math.abs(power) * dir, Range.clip(turnMod, -0.2, 0.2), 1);
        }
        robot.driveTrain.stopMove();
    }


    protected void turnToHeading(double targetHeading, double power) {
        while (opModeIsActive() && Math.abs(getHeadingDiff(targetHeading)) > 6) { // added opMode=true &&
            robot.driveTrain.startMove(0, 0, 1, power * Math.signum(getHeadingDiff(targetHeading)));
        }
        robot.driveTrain.stopMove();
    }

    protected void spikeDump() {
        robot.lift.motorLift.setTargetPosition(robot.lift.LIFT_HOVER_POS);
        robot.lift.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLift.setPower(1);
        sleep(500);
        robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_OUTAKE_POS);
        sleep(1000);
        robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_INIT_POS);
        sleep(1000);
        robot.lift.motorLift.setTargetPosition(robot.lift.LIFT_INTAKE_POS);
        robot.lift.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLift.setPower(1);
        sleep(500);
    }

    protected void liftPos(int height) {
        robot.lift.motorLift.setTargetPosition(height);
        robot.lift.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLift.setPower(1);
        sleep(2000);

    }

    protected void dump() {
        robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_OUTAKE_POS);
        sleep(1000);
        robot.lift.servoBucket.setPosition(robot.lift.SERVO_BUCKET_INTAKE_POS);
    }

    protected void intakeOn(int ms) {
        robot.lift.motorIntakeLowSpeed.setPower(robot.lift.INTAKE_LS_POWER);
        robot.lift.motorIntakeHighSpeed.setPower(robot.lift.INTAKE_HS_POWER);
        sleep(ms);
        robot.lift.motorIntakeLowSpeed.setPower(0);
        robot.lift.motorIntakeHighSpeed.setPower(0);
    }


}
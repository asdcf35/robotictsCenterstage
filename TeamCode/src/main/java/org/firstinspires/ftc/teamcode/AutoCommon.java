package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;
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
import org.openftc.easyopencv.OpenCvPipeline;

//public class AutoCommon extends LinearOpMode {
//    protected RobotHardware robot;
//
//       public void runOpMode() {
//
//        telemetry.addData("Status", "Initializing...");
//        telemetry.update();
//
//        cameraMagic();
//        telemetry.update();
//
//        robot = new RobotHardware(hardwareMap, true, true, true, true);
//        initialHeading = getHeading();
//        telemetry.update();
//
//
//        telemetry.addData("Status", "Initialized...");
//        telemetry.update();
//
//
//        waitForStart();
//        telemetry.update();
//
//        // Release resources
//    }
//    double cX = 0;
//    double cY = 0;
//    double width = 0;
//
//    private OpenCvCamera camera;  // Use OpenCvCamera class from FTC SDK
//    private static final int CAMERA_WIDTH = 640; // width of wanted camera resolution
//    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
//
//    private static int[][] ROIs = {
//            {122, 174, 340, 200},
//            {465, 141, 615, 184}
//    };
//
//    // Calculate the distance using the formula
//    //122:174
//    //340:200
//
//    //right
//
//    //465:141
//    //615:184
//    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
//    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
//
//    int error = 60;
//    int pos = 1;
//    public void cameraMagic() {
//
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(camera, 30);
//        initOpenCV();
//
//        while (opModeInInit()) {
//            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
//
//            telemetry.update();
//            // The OpenCV pipeline automatically processes frames and handles detection
//        }
//    }
//    private  void initOpenCV() {
//
//        // Create an instance of the camera
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
//        camera = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//            }
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//        camera.setPipeline(new YellowBlobDetectionPipeline());
//    }
//
//    private Mat getROI(int x1, int y1, int x2, int y2) {
//        Mat frame = new Mat();
//        Rect roi = new Rect(x1, y1, x2, y2); // Define the ROI (x, y, width, height)
//        Mat roiFrame = new Mat(frame, roi);
//
//        return roiFrame;
//    }
//
//
//}



public class AutoCommon extends LinearOpMode {
    protected RobotHardware robot;

    public void runOpMode() {

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        cameraMagic();
        telemetry.update();

        robot = new RobotHardware(hardwareMap, true, true, true, true);
        initialHeading = getHeading();
        telemetry.update();


        telemetry.addData("Status", "Initialized...");
        telemetry.update();


        waitForStart();
        telemetry.update();

        // Release resources
    }
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera camera;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private static int[][] ROIs = {
            {122, 174, 340, 200},
            {465, 141, 615, 184}
    };

    // Calculate the distance using the formula
    //122:174
    //340:200

    //right

    //465:141
    //615:184
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    int error = 60;
    int pos = 1;
    public void cameraMagic() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 30);
        initOpenCV();

        while (opModeInInit()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            if (cX <= 225+error && cX >= 225-error) { //change 227 to the cX value when piece is in center
                telemetry.addData("Location: ", "Center");
                pos = 2;
                //                center();
            } else if (cX <= 415+error && cX >= 415-error ) {//change 60 to the cX value when piece is in center
                telemetry.addData("Location", "Right");
                //                right();
                pos = 3;
            } else {
                telemetry.addData("Location:", "Left");
                //                left();
                pos = 1;
            }
            telemetry.update();
            // The OpenCV pipeline automatically processes frames and handles detection
        }
    }
    private  void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        camera.setPipeline(new YellowBlobDetectionPipeline());
    }

    private Mat getROI(int x1, int y1, int x2, int y2) {
        Mat frame = new Mat();
        Rect roi = new Rect(x1, y1, x2, y2); // Define the ROI (x, y, width, height)
        Mat roiFrame = new Mat(frame, roi);

        return roiFrame;
    }

    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow region
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(40, 40));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }

        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }

    private static double getDistance(double width) {
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
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
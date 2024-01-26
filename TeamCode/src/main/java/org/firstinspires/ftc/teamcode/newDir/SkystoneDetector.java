package org.firstinspires.ftc.teamcode.newDir;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }
    private Location location;
    static final Rect LEFT_ROI = new Rect(
        new Point(60, 35),
        new Point(120, 75)
    );
    static final Rect MIDDLE_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75)
    );
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75)
    );
    static double PERCENT_COLOR_THRESHOLD = 0.4;
    public SkystoneDetector(Telemetry t){telemetry = t;}
    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        double leftValue = Core.sumElems(left).val[0]/LEFT_ROI.area()/255;
        double rightValue = Core.sumElems(right).val[0]/RIGHT_ROI.area()/255;
        double middleValue = Core.sumElems(middle).val[0]/MIDDLE_ROI.area()/255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");

        boolean spikeRight = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean spikeLeft = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean spikeMiddle = middleValue > PERCENT_COLOR_THRESHOLD;


        if (spikeMiddle){
            location = Location.MIDDLE;
            telemetry.addData("Centerstage Location","middle");
        }
        else if(spikeLeft){
            location = Location.LEFT;
            telemetry.addData("Centerstage Location", "right");
        }
        else if(spikeRight){
            location = Location.RIGHT;
            telemetry.addData("Centerstage Location", "left");
        }
        else{
            location = Location.NOT_FOUND;
            telemetry.addData("Centerstage Location", "not found");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        telemetry.update();

        Imgproc.cvtColor(mat, mat,  Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);
        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? colorSkystone:colorStone);
        return mat;
    }


    public Location getLocation() {
        return location;
    }
}

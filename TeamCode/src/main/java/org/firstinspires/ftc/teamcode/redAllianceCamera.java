package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "drive")
public class redAllianceCamera extends AutoCommon {

    YellowBlobDetectionPipeline yellowBlobDetectionPipeline;
    OpenCvCamera camera;

    @Override
    public void runOpMode()
    {
        super.runOpMode();


        if (pos == 2) {
            driveOnHeading(20, 0.3, 0);
        }

        else if (pos == 1){
            driveOnHeading(20, 0.3, 90);
        }

        else if (pos == 3){
            driveOnHeading(22, 0.3, -90);
        }

    }
}
package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ConeDetectorVisionPipeline extends OpenCvPipeline
{
    Telemetry telemetry;
    FtcDashboard dashboard;

    // Color constants to use later
    static final Scalar BLUE = new Scalar(0, 0, 255);

    Mat greyscale = new Mat();
    MatOfPoint largestContour;

    // Volatile since accessed by OpMode thread without synchronization
    private volatile int coneXPos = 0;

    public ConeDetectorVisionPipeline(Telemetry telemetry, FtcDashboard dashboard) {
        this.telemetry = telemetry;
        this.dashboard = dashboard;
    }

    void inputToGreyscale(Mat input) {
        Imgproc.cvtColor(input, greyscale, Imgproc.COLOR_RGB2GRAY);
    }

    @Override
    public void init(Mat firstFrame)
    {
        inputToGreyscale(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Mat binary = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));
        Imgproc.threshold(greyscale, binary, 100, 255, Imgproc.THRESH_BINARY_INV);
        //Finding Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);

        int largest = 0;
        double largestArea = 0;
        for (int i = 0; i < contours.size(); i++) {
            //Calculating the area
            double cont_area = Imgproc.contourArea(contours.get(i));
            telemetry.addData("Countour Area: ", cont_area);
            if (cont_area > largestArea) {
                largest = i;
            }
            Imgproc.drawContours(input, contours, i, BLUE, 2,
                    Imgproc.LINE_8, hierarchy, 2, new Point());
        }
        largestContour = contours.get(largest);
        coneXPos = (int) (Imgproc.moments(largestContour).get_m10() / Imgproc.moments(largestContour).get_m00());

        return input;
    }

    public int getAnalysis()
    {
        return coneXPos;
    }
}
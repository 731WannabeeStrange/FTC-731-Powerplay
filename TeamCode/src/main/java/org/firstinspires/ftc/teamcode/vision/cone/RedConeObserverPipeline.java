package org.firstinspires.ftc.teamcode.vision.cone;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedConeObserverPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Mat hsvMat = new Mat();

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        if (hsvMat.empty()) {
            return input;
        }

        Scalar lowHSV = new Scalar(160, 70, 80);
        Scalar highHSV = new Scalar(195, 255, 255);

        Mat thresh = new Mat();

        Core.inRange(hsvMat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        Core.bitwise_and(hsvMat, hsvMat, masked, thresh);

        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        masked.convertTo(scaledMask, -1, 150/average.val[1], 0);

        Scalar strictLowHSV = new Scalar(0, 150, 100);
        Scalar strictHighHSV = new Scalar(255, 255, 255);

        Mat scaledThresh = new Mat();
        Core.inRange(scaledMask, strictLowHSV,strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        Core.bitwise_and(hsvMat, hsvMat, finalMask, scaledThresh);

        Mat edges = new Mat();
        Imgproc.Canny(finalMask,edges, 100, 200);

        // Release everything
        thresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        hsvMat.release();
        masked.release();
        edges.release();
        thresh.release();
        finalMask.release();

        return input;
    }
}

package org.firstinspires.ftc.teamcode.mmintothedeep.util.Camera.eocv1;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SimpleThresholdingTest extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Scalar lower_blue = new Scalar(85, 50, 40);
        Scalar upper_blue = new Scalar(135, 255, 255);

        Scalar lower_red = new Scalar(0, 125, 80);
        Scalar upper_red = new Scalar(10, 255, 255);

        // Brown => (40, 20, 90);
        // Brown => (255, 70, 255);

        // Both blue and yellow Scalar lower_red = new Scalar(20, 100, 100);
        // Both blue and yellow Scalar upper_red = new Scalar(135, 255, 255);

        Scalar lower_yellow = new Scalar(20, 100, 100);
        Scalar upper_yellow = new Scalar(30, 255, 255);
        // GBR
        //Convert RGB to HSV
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        //Apply Threshold
        Mat threshold = new Mat();
        // Core.inRange(hsvMat, lower_blue, upper_blue, threshold);
        Core.inRange(hsvMat, lower_red, upper_red, threshold);
        // Core.inRange(hsvMat, lower_yellow, upper_yellow, threshold);

        return threshold;
    }
}
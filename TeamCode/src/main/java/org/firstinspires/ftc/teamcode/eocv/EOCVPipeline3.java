package org.firstinspires.ftc.teamcode.eocv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class EOCVPipeline3 extends OpenCvPipeline {
    public Scalar lowHSV = new Scalar(0, 162.9, 97.8);
    public Scalar highHSV = new Scalar(178, 211, 110);

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }



        Mat thresh = new Mat();

        Core.inRange(mat, lowHSV, highHSV, thresh);


        return thresh;

    }
}







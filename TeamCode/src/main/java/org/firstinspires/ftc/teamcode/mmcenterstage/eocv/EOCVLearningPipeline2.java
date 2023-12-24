package org.firstinspires.ftc.teamcode.mmcenterstage.eocv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class EOCVLearningPipeline2 extends OpenCvPipeline {
    Mat mat = new Mat();
    Mat thresh = new Mat();
    Mat leftCrop = new Mat();
    Mat centerCrop = new Mat();
    Mat rightCrop = new Mat();

    public Scalar lowHSV = new Scalar(252,73,7);
    public Scalar highHSV = new Scalar(253,135,61);


    @Override
    public Mat processFrame(Mat input) {


        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }


        Core.inRange(mat, lowHSV, highHSV, thresh);

            /*

            -----------------------------------
                         UNFINISHED
            -----------------------------------


             */


        return thresh;
    }
}

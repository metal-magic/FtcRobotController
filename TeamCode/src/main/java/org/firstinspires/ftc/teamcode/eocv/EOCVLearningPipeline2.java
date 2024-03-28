package org.firstinspires.ftc.teamcode.eocv;

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

    public Scalar lowHSV = new Scalar(56.7,106.3,49.6);
    public Scalar highHSV = new Scalar(255,239,221);

    Rect leftRect = new Rect(1, 1, 213, 479);
    Rect centerRect = new Rect(213, 1, 213, 479);
    Rect rightRect = new Rect(426, 1, 213, 479);

    @Override
    public Mat processFrame(Mat input) {


        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }


        Core.inRange(mat, lowHSV, highHSV, thresh);


        Imgproc.rectangle(thresh, leftRect, new Scalar(255,255,255), 2);
        Imgproc.rectangle(thresh, centerRect, new Scalar(255,255,255), 2);
        Imgproc.rectangle(thresh, rightRect, new Scalar(255,255,255), 2);

        leftCrop = thresh.submat(leftRect);
        centerCrop = thresh.submat(centerRect);
        rightCrop = thresh.submat(rightRect);

            /*

            -----------------------------------
                         UNFINISHED
            -----------------------------------


             */


        return thresh;
    }
}

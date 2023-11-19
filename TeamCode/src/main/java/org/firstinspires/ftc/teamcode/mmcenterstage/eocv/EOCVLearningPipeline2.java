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

    Scalar lowHSV = new Scalar(253,135,61);
    Scalar highHSV = new Scalar(252,73,7);

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }


        Core.inRange(mat, lowHSV, highHSV, thresh);

        Rect leftRect = new Rect(1, 1, 639, 1071);
        Rect centerRect = new Rect(640, 1, 639, 1071);
        Rect rightRect = new Rect(1279, 1, 639, 1071);

        Imgproc.rectangle(thresh, leftRect, new Scalar(255,255,255), 2);
        Imgproc.rectangle(thresh, centerRect, new Scalar(255,255,255), 2);
        Imgproc.rectangle(thresh, rightRect, new Scalar(255,255,255), 2);

        Mat leftCrop = thresh.submat(leftRect);
        Mat centerCrop = thresh.submat(centerRect);
        Mat rightCrop = thresh.submat(rightRect);

            /*

            -----------------------------------
                         UNFINISHED
            -----------------------------------


             */


        return thresh;
    }
}

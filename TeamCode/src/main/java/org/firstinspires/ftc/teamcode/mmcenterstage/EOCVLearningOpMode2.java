package org.firstinspires.ftc.teamcode.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class EOCVLearningOpMode2 extends OpMode {

    // create webcam as a class member
    OpenCvWebcam webcam = null;


    @Override
    public void init() {

        // initializing the camera
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "testWebcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // initializing the pipeline
        webcam.setPipeline(new EOCVLearningPipeline2());

        // open webcam
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Camera opened successfully");
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("ERROR: Could not open camera");
            }
        });


    }

    @Override
    public void loop() {

    }


    class EOCVLearningPipeline2 extends OpenCvPipeline {

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
}
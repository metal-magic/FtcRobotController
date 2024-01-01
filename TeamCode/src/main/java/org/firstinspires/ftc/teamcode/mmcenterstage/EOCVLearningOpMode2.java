package org.firstinspires.ftc.teamcode.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mmcenterstage.eocv.EOCVLearningPipeline2;
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
@TeleOp
@Disabled
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


}
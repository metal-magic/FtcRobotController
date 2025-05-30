//package org.firstinspires.ftc.teamcode.mmintothedeep.util.Camera;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.Date;
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
//
//import android.util.Size;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
//import org.firstinspires.ftc.vision.opencv.ColorRange;
//import org.firstinspires.ftc.vision.opencv.ImageRegion;
//import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
//import org.opencv.core.RotatedRect;
//import org.opencv.core.Point;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//
///*
// * This OpMode determines the best Exposure for minimizing image motion-blur on a Webcam
// * Note that it is not possible to control the exposure for a Phone Camera, so if you are using a Phone for the Robot Controller
// * this OpMode/Feature only applies to an externally connected Webcam
// *
// * The goal is to determine the smallest (shortest) Exposure value that still provides reliable Tag Detection.
// * Starting with the minimum Exposure and maximum Gain, the exposure is slowly increased until the Tag is
// * detected reliably from the likely operational distance.
// *
// *
// * The best way to run this optimization is to view the camera preview screen while changing the exposure and gains.
// *
// * To do this, you need to view the RobotController screen directly (not from Driver Station)
// * This can be done directly from a RC phone screen (if you are using an external Webcam), but for a Control Hub you must either plug an
// * HDMI monitor into the Control Hub HDMI port, or use an external viewer program like ScrCpy (https://scrcpy.org/)
// *
// * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
// */
//
//@Autonomous//(name="Align camera to colored sample", group = "Concept")
//// @Disabled
////@Disabled
//public class ColorAutonomous extends LinearOpMode
//{
//    private VisionPortal visionPortal = null;        // Used to manage the video source.
//    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
//    private ColorBlobLocatorProcessor colorLocator;
//    private int myExposure;
//    private int myGain;
//
//    /* Declare all motors as null */
//    Date currentTime = new Date();
//    Servo gripperServo1 = null;
//    Servo pivotServo = null;
//    CRServo armMotor = null;
//
//    /*
//     * ==============================================================
//     *
//     * below values also available in UtilityValues Class
//     *
//     * ==============================================================
//     */
//    static final double MOTOR_TICK_COUNTS = UtilityValues.motorTicks; // goBILDA 5203 series Yellow Jacket
//    // figure out how many times we need to turn the wheels to go a certain distance
//    // the distance you drive with one turn of the wheel is the circumference of the wheel
//    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
//    static final double WHEEL_DIAMETER_INCHES = UtilityValues.wheelDiameter / 25.4; // in Inches
//    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches
//    static final double DEGREES_MOTOR_MOVES_IN_1_REV = UtilityValues.COMPETITION_MOTOR_MOVES_IN_1_REV;
//    static final double SPEED = UtilityValues.SPEED; // Motor Power setting
//
//    private boolean USE_WEBCAM = true;
//    private boolean REFRESH_WEBCAM = false;
//
//    private boolean aligned = false;
//    private boolean direction = false;
//
//    @Override public void runOpMode()
//    {
//        // Initialize the Apriltag Detection process
//        // initAprilTag();
//        // initColorProcessor();
//        // initColorBlobsProcessor();
//
//        initMotor();
//
//        initColorBlobsProcessor(ColorRange.BLUE);
//
//
////        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
////                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
////                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
////                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0, 0.5, -1))  // search central 1/4 of camera view
////                // .setDrawContours(true) (DO NOT UNCOMMENT)                       // Show contours on the Stream Preview
////                .setBlurSize(3)                               // Smooth the transitions between different colors in image
////                //.setErodeSize(6)
////                //.setDilateSize(6)
////                .build();
////
////        visionPortal = new VisionPortal.Builder()
////                .addProcessor(colorLocator)
////                .setCameraResolution(new Size(640, 480))
////                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
////                .build();
//
//        /*ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.6875, 0.7083, 0.6875, -0.7083))  // search central 1/4 of camera view
//                // .setDrawContours(true)                        // Show contours on the Stream Preview
//                .setBlurSize(5)                               // Smooth the transitions between different colors in image
//                //.setErodeSize(6)
//                //.setDilateSize(6)
//                .build();
//
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(colorLocator)
//                .setCameraResolution(new Size(640, 480))
//                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
//                .build();*/
//
//
//        // Establish Min and Max Gains and Exposure.  Then set a low exposure with high gain
//        getCameraSetting();
//        // myExposure = Math.min(5, minExposure);
//        myExposure = 26;
//        myGain = 200;
//        setManualExposure(myExposure, myGain);
//
//        // Wait for the match to begin.
//        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch START to start OpMode");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            telemetry.addLine("Find lowest Exposure that gives reliable detection.");
//            telemetry.addLine("Use Left bump/trig to adjust Exposure.");
//            telemetry.addLine("Use Right bump/trig to adjust Gain.\n");
//
//            // streamWebcamRefresh();
//
//            // Display how many Tags Detected
//            /*
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            int numTags = currentDetections.size();
//            if (numTags > 0 )
//                telemetry.addData("Tag", "####### %d Detected  ######", currentDetections.size());
//            else
//                telemetry.addData("Tag", "----------- none - ----------");
//            */
////            telemetry.addData("Exposure","%d  (%d - %d)", myExposure);
////            telemetry.addData("Gain","%d  (%d - %d)", myGain);
////            telemetry.addData("DS preview on/off", "3 dots, Camera Stream\n");
//
//            // Request the most recent color analysis.
//            // This will return the closest matching colorSwatch and the predominant RGB color.
//            // Note: to take actions based on the detected color, simply use the colorSwatch in a comparison or switch.
//            //  eg:
//            //      if (result.closestSwatch == PredominantColorProcessor.Swatch.RED) {... some code  ...}
//
//            // PredominantColorProcessor.Result result = colorLocator.getAnalysis();
//
//            // Display the Color Sensor result.
//            //  telemetry.addData("Best Match:", result.closestSwatch);
//            //  telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
//            //  telemetry.update();
//
//            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
//            ColorBlobLocatorProcessor.Util.filterByArea(500, 10000, blobs);
//
//            sleep(200);
//
////            for(ColorBlobLocatorProcessor.Blob b : blobs) {
////                RotatedRect boxFit = b.getBoxFit();
////                telemetry.addLine(String.valueOf(b.getContourArea()));
////                telemetry.addLine(String.valueOf(b.getDensity()));
////                telemetry.addLine(String.valueOf(b.getAspectRatio()));
////                telemetry.addLine(String.valueOf((int) boxFit.center.x));
////                telemetry.addLine(String.valueOf((int) boxFit.center.y));
////                if (Math.abs(((int) boxFit.center.x) - 320) <= 20) {
////                    telemetry.addLine("CENTERED");
////                }
////                if ((int) boxFit.center.x < 280) {
////                    strafe(-1);
////                }
////                else if ((int) boxFit.center.x > 360) {
////                    strafe(1);
////                }
////            }
//
////            org.opencv.core.Size myBoxFitSize;
////            if (!blobs.isEmpty()) {
////                RotatedRect boxFit = blobs.get(0).getBoxFit();
////                myBoxFitSize = boxFit.size;
////                double boxWidth = myBoxFitSize.width;
////                double boxHeight = myBoxFitSize.height;
////                int currX = (int) boxFit.center.x;
////                double error = 320 - currX;
////                int angle = (int) boxFit.angle;
////                if (Math.abs((currX) - 320) <= 20) {
////                    telemetry.addLine("X CENTERED");
////                    strafe(5);
////                }
////                else if (Math.abs((currX) - 320) <= 100) {
////                    strafe(-1 * error/40);
////                }
////                else {
////                    strafe(-1 * error/20);
////
////                }
////                telemetry.addLine(String.valueOf((int) boxFit.center.x));
////                telemetry.addLine(String.valueOf(18644/Math.min(boxHeight, boxWidth)));
////                for (ColorBlobLocatorProcessor.ContourMode c : ColorBlobLocatorProcessor.ContourMode.values())
////                    telemetry.addLine(String.valueOf(c));
////            }
//
//            //
//
//            org.opencv.core.Size myBoxFitSize;
//            double strafeX = 1;
//            if (!blobs.isEmpty() && !aligned) {
//                RotatedRect boxFit = blobs.get(0).getBoxFit();
//                Point[] myBoxCorners = new Point[4];
//                boxFit.points(myBoxCorners);
//                // this points() method does not return values, it populates the argument
//                double minLength = 2500;
//                int point = 0;
//                for (int i = 0; i <= 3; i++)
//                {
//                    double adjacentLength = Math.sqrt(Math.pow(Math.abs((int) myBoxCorners[i%4].x - (int) myBoxCorners[(i+1)%4].x), 2) + Math.pow(Math.abs((int) myBoxCorners[i%4].y - (int) myBoxCorners[(i+1)%4].y), 2));
//                    if (adjacentLength < minLength) {
//                        minLength = adjacentLength;
//                        point = i;
//                    }
//                }
//                // Distance in inches
//                double distanceZ = 734.01575 / minLength;
//                int midX = (int) ((myBoxCorners[point].x+myBoxCorners[point+1].x)/2);
//                int midY = (int) ((myBoxCorners[point].y+myBoxCorners[point+1].y)/2);
//                double distanceX = (midX - 320);
//                double distanceY = (240 - midY);
//                strafeX = distanceX * distanceZ / (Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2)));
//                double moveY = distanceY * distanceZ / (Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2)));
//                strafe(strafeX);
//                sleep(500);
//                if (Math.abs(boxFit.center.x - 320) <= 20) {
//                    aligned = true;
//                }
//            } else {
//                strafe(-0.5 * strafeX);
//            }
//
//            telemetry.addLine(String.valueOf(aligned));
//
//
//            sleep(20);
//            telemetry.update();
//
//        }
//    }
//
//    /**
//     * Initialize the AprilTag processor.
//     */
//    private void initAprilTag() {
//        // Create the AprilTag processor by using a builder.
//        aprilTag = new AprilTagProcessor.Builder().build();
//
//        // Create the WEBCAM vision portal by using a builder.
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
//                .addProcessor(aprilTag)
//                .build();
//    }
//
//
//
//    /**
//     * Initialize the ColorSensor processor
//     */
//
//    public void initColorProcessor() {
//        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.2, 0.2, 0.2, -0.2))
//                .setSwatches(
//                        PredominantColorProcessor.Swatch.RED,
//                        PredominantColorProcessor.Swatch.BLUE,
//                        PredominantColorProcessor.Swatch.BLACK,
//                        PredominantColorProcessor.Swatch.YELLOW)
//                .build();
//
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(colorSensor)
//                .setCameraResolution(new Size(640, 480))
//                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
//                .build();
//    }
//
//    /**
//     * Initialize the ColorLocator processor
//     */
//
//    public void initColorBlobsProcessor(ColorRange color) {
//        colorLocator = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(color)         // use a predefined color match
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.25 , 0.5, -1))  // search central 1/4 of camera view
//                // .setDrawContours(true)                        // Show contours on the Stream Preview
//                .setBlurSize(5)                               // Smooth the transitions between different colors in image
//                .setErodeSize(3)
//                .setDilateSize(2)
//                .build();
//
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(colorLocator)
//                .setCameraResolution(new Size(640, 480))
//                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
//                .build();
//    }
//
////    public void autoAlign() {
////
////
////    }
//
//
//    /**
//     * streams camera image to DS
//     */
////    private void streamWebcamRefresh() {
////        // Save CPU resources; can resume streaming when needed.
////        visionPortal.resumeStreaming();
////        // Share the CPU.
////        sleep(20);
////    }
//
//
//
//    /*
//        Manually set the camera gain and exposure.
//        Can only be called AFTER calling initAprilTag();
//        Returns true if controls are set.
//     */
//    private boolean setManualExposure(int exposureMS, int gain) {
//        // Ensure Vision Portal has been setup.
//        if (visionPortal == null) {
//            return false;
//        }
//
//        // Wait for the camera to be open
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        // Set camera controls unless we are stopping.
//        if (!isStopRequested())
//        {
//            // Set exposure. Make sure we are in Manual Mode for these values to take effect.
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);
//
//            // Set Gain.
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            sleep(20);
//            return (true);
//        } else {
//            return (false);
//        }
//    }
//
//    private void getCameraSetting() {
//        // Ensure Vision Portal has been setup.
//        if (visionPortal == null) {
//            return;
//        }
//
//        // Wait for the camera to be open
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//    }
//
//
//}

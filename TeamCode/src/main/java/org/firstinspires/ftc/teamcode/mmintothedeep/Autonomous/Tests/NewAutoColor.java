package org.firstinspires.ftc.teamcode.mmintothedeep.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.mmintothedeep.util.UtilityValues;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.SortOrder;

@Autonomous
@Disabled
public class NewAutoColor extends LinearOpMode {
    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private ColorBlobLocatorProcessor colorLocator;
    private int myExposure;
    private int myGain;

    /* Declare all motors as null */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    Servo gripperServo1 = null;
    Servo pivotServo = null;
    CRServo armMotor = null;

    /*
     * ==============================================================
     *
     * below values also available in UtilityValues Class
     *
     * ==============================================================
     */
    static final double MOTOR_TICK_COUNTS = UtilityValues.motorTicks; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER_INCHES = UtilityValues.wheelDiameter / 25.4; // in Inches
    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches
    static final double DEGREES_MOTOR_MOVES_IN_1_REV = UtilityValues.COMPETITION_MOTOR_MOVES_IN_1_REV;
    static final double SPEED = UtilityValues.SPEED; // Motor Power setting

    private boolean USE_WEBCAM = true;
    private boolean REFRESH_WEBCAM = false;


    static final double MAX_PIVOT_DISTANCE_INCHES = 10;


    @Override public void runOpMode() {

        initMotor();

        initColorBlobsProcessor(ColorRange.YELLOW);

        getCameraSetting();
        myExposure = 30;
        myGain = 230;
        setManualExposure(myExposure, myGain);

        waitForStart();

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ASSUMES THE ROBOT STARTS NEAR THE BASKET FACING THE 3 SAMPLES
        alignToSample();
        // Pick up sample and rotate
        //pickUpSample();
    }


    // Method to initialize all motors
    public void initMotor() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Set all the right motor directions
        leftFrontDrive.setDirection(UtilityValues.finalLeftFrontDirection);
        leftBackDrive.setDirection(UtilityValues.finalLeftBackDirection);
        rightFrontDrive.setDirection(UtilityValues.finalRightFrontDirection);
        rightBackDrive.setDirection(UtilityValues.finalRightBackDirection);

        // Reset encoders positions
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Declares that encoders will be used
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Sets default gripperServo position
        //gripperServo1.setPosition(1);

        // Sets default linearSlide position
        //linearSlide.setPosition(0);
    }

    // Method to initialize colorBlobProcessor (Drawing boxes around samples)
    public void initColorBlobsProcessor(ColorRange color) {
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.25 , 0.75, -1))  // search central 1/4 of camera view
                // .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .setErodeSize(3)
                .setDilateSize(2)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .build();
    }

    // Method to manually set exposure and gain values
    private boolean setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure. Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(5);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(2);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(10);
            return (true);
        } else {
            return (false);
        }
    }

    // Method to stream camera frames to the driver station
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }

    private void alignToSample() {
        // Robot is misaligned to begin with
        boolean alignedX = false;
        boolean alignedY = false;
        int maxRepetitions = 5;
        // Allows while loops below to access boxFitSize
        org.opencv.core.Size myBoxFitSize;
        int i = 0;
        double errorX;


        while (!alignedX && i < maxRepetitions) {
            // Blobs is an arrayList of type ColorBlobLocatorProcessor
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
//            // Filters by AspectRatio to remove wall when detecting yellow
            ColorBlobLocatorProcessor.Util.filterByAspectRatio(1, 2, blobs);
            // Filters by Area to remove small, glitched blobs
            ColorBlobLocatorProcessor.Util.filterByArea(500, 10000, blobs);
            // Sorts by Area in descending order to make processing easier
            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);



            if (!blobs.isEmpty()) {
                // Assigned boxFit to the largest detect blob
                RotatedRect boxFit = blobs.get(0).getBoxFit();

                errorX = boxFit.center.x - 320;

                telemetry.addLine(String.valueOf(errorX));

                // V1 HORIZONTAL ALIGNMENT
//                if (errorX > 0) {
//                    strafe(2 - 2 / (1 + Math.pow(100000, ((double) i / maxRepetitions + 0.5))));
//                } else {
//                    strafe(-1 * (2 - 2 / (1 + Math.pow(100000, ((double) i / maxRepetitions + 0.5)))));
//                }

//                // V2 HORIZONTAL ALIGNMENT
//                strafe(Math.signum(errorX) * 3 * (1-Math.pow(((double) i/maxRepetitions), 0.1)));


//                // V3 HORIZONTAL ALIGNMENT
//                if (current == 0) {
//                    if (errorX > 0) {
//                        upper += 5;
//                    } else {
//                        lower -= 5;
//                    }
//                } else {
//                    if (errorX > 0) {
//                        lower = current;
//                    } else {
//                        upper = current;
//                    }
//                }
//
//                current = (upper+lower)/2-current;
//                strafe(current);

                // strafe(Math.signum(errorX) * (1-1/(1+Math.pow(100000, ((double) (i / maxRepetitions + 0.5)))));

                strafe(Math.signum(errorX) * 3 * Math.pow(2, -1 * i));

                alignedX = Math.abs(errorX) <= 30;
            }
            i++;
            sleep(50);
        }

//        // Sample horizontal setting
//
//        // Blobs is an arrayList of type ColorBlobLocatorProcessor
//        List<ColorBlobLocatorProcessor.Blob> blobsX1 = colorLocator.getBlobs();
//        // Filters by AspectRatio to remove wall when detecting yellow
//        ColorBlobLocatorProcessor.Util.filterByAspectRatio(1, 5, blobsX1);
//        // Filters by Area to remove small, glitched blobs
//        ColorBlobLocatorProcessor.Util.filterByArea(500, 10000, blobsX1);
//        // Sorts by Area in descending order to make processing easier
//        // ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
//
//        double P1 = 320;
//        if (!blobsX1.isEmpty()) {
//
//            RotatedRect boxFit = blobsX1.get(0).getBoxFit();
//            P1 = boxFit.center.x - 320;
//        }
//
//        // Sets up ratio between horizontal distance and vertical distance
//        strafe(Math.signum(P1) * strafeDistance);
//
//        // Automatic Horizontal Alignment
//
//        // Blobs is an arrayList of type ColorBlobLocatorProcessor
//        List<ColorBlobLocatorProcessor.Blob> blobsX2 = colorLocator.getBlobs();
//        // Filters by AspectRatio to remove wall when detecting yellow
//        ColorBlobLocatorProcessor.Util.filterByAspectRatio(1, 5, blobsX2);
//        // Filters by Area to remove small, glitched blobs
//        ColorBlobLocatorProcessor.Util.filterByArea(500, 10000, blobsX2);
//        // Sorts by Area in descending order to make processing easier
//        // ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
//
//        double P2 = 320;
//        if (!blobsX2.isEmpty()) {
//
//            RotatedRect boxFit = blobsX2.get(0).getBoxFit();
//
//            P2 = boxFit.center.x - 320;
//        }
//
//        strafe((P1 * strafeDistance) / (P1 - P2));
//
//
//        sleep(500);
//
//
        // Blobs is an arrayList of type ColorBlobLocatorProcessor
        List<ColorBlobLocatorProcessor.Blob> blobsY = colorLocator.getBlobs();
        // Filters by AspectRatio to remove wall when detecting yellow
        ColorBlobLocatorProcessor.Util.filterByAspectRatio(1, 2, blobsY);
        // Filters by Area to remove small, glitched blobs
        ColorBlobLocatorProcessor.Util.filterByArea(500, 10000, blobsY);
        // Sorts by Area in descending order to make processing easier
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobsY);

        if (!blobsY.isEmpty()) {

            RotatedRect boxFit = blobsY.get(0).getBoxFit();
            myBoxFitSize = boxFit.size;
            double boxWidth = myBoxFitSize.width;
            double boxHeight = myBoxFitSize.height;

            double distanceZ_INCHES = 734.01575 / Math.min(boxHeight, boxWidth);
            double errorY = distanceZ_INCHES - MAX_PIVOT_DISTANCE_INCHES;

//                if (errorY > 0) {
//                    moveStraightLine(2 - 2 / (1 + Math.pow(100000, ((double) j / maxRepetitions + 0.5))));
//                } else {
//                    moveStraightLine(-1 * (2 - 2 / (1 + Math.pow(100000, ((double) j / maxRepetitions + 0.5)))));
//                }
            moveStraightLine(errorY);

            // moveStraightLine(Math.signum(errorX) * (1-1/(1+Math.pow(100000, ((double) (j / maxRepetitions + 0.5)))));

            alignedY = Math.abs(errorY) <= 0.1;
        }

    }

    private void alignToSample2 () {
        boolean alignedX = false;
        int maxRepetitions = 5;
        int i = 0;
        while (!alignedX && i < maxRepetitions) {
            // Blobs is an arrayList of type ColorBlobLocatorProcessor
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            // Filters by AspectRatio to remove wall when detecting yellow
            ColorBlobLocatorProcessor.Util.filterByAspectRatio(1, 5, blobs);
            // Filters by Area to remove small, glitched blobs
            ColorBlobLocatorProcessor.Util.filterByArea(500, 10000, blobs);
            // Sorts by Area in descending order to make processing easier
            // ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

            if (!blobs.isEmpty()) {
                RotatedRect boxFit = blobs.get(0).getBoxFit();

                double errorX = boxFit.center.x - 320;

                strafe(Math.signum(errorX) * 2 * (1-Math.pow(((double) i/maxRepetitions), 0.1)));

                alignedX = Math.abs(errorX) <= 30;
            } else {
                strafe(-2);
            }
            i++;
        }

        // Blobs is an arrayList of type ColorBlobLocatorProcessor
        List<ColorBlobLocatorProcessor.Blob> blobsY = colorLocator.getBlobs();
        // Filters by AspectRatio to remove wall when detecting yellow
        ColorBlobLocatorProcessor.Util.filterByAspectRatio(1, 5, blobsY);
        // Filters by Area to remove small, glitched blobs
        ColorBlobLocatorProcessor.Util.filterByArea(500, 10000, blobsY);
        // Sorts by Area in descending order to make processing easier
        // ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

        // Allows while loops below to access boxFitSize
        org.opencv.core.Size myBoxFitSize;
        if (!blobsY.isEmpty()) {

            RotatedRect boxFit = blobsY.get(0).getBoxFit();
            myBoxFitSize = boxFit.size;
            double boxWidth = myBoxFitSize.width;
            double boxHeight = myBoxFitSize.height;

            double distanceZ_INCHES = 734.01575 / Math.min(boxHeight, boxWidth);
            double errorY = distanceZ_INCHES - MAX_PIVOT_DISTANCE_INCHES;
            moveStraightLine(errorY);
        }
    }

    private void alignToSampleX () {
        boolean alignedX = false;
        int maxRepetitions = 5;
        int i = 0;
        while (!alignedX && i < maxRepetitions) {
            // Blobs is an arrayList of type ColorBlobLocatorProcessor
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            // Filters by AspectRatio to remove wall when detecting yellow
            ColorBlobLocatorProcessor.Util.filterByAspectRatio(1, 5, blobs);
            // Filters by Area to remove small, glitched blobs
            ColorBlobLocatorProcessor.Util.filterByArea(500, 10000, blobs);
            // Sorts by Area in descending order to make processing easier
            // ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

            if (!blobs.isEmpty()) {
                RotatedRect boxFit = blobs.get(0).getBoxFit();

                double errorX = boxFit.center.x - 320;

                strafe(Math.signum(errorX) * Math.log(errorX));

                alignedX = Math.abs(errorX) <= 30;
            }
            i++;
        }
    }

    public void pickUpSample() {
        // pivotServo.setPosition(0.36);
        // gripperServo.setPosition(0.3);
        // pivotServo.setPosition(0.85);
        // rotate(-90);
    }

    public void rotate(double degrees) {

        double robotSpeed = SPEED;
        // Assume positive degrees means moving towards the right
        double movementOfWheelsInRevs = Math.abs(degrees / DEGREES_MOTOR_MOVES_IN_1_REV);

        if (degrees >= 0) {
            drive(robotSpeed,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs);
        } else {
            // Moving negative means rotating left
            drive(robotSpeed,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs);
        }
    }

    private void strafe(double strafeInches) {
        // We assume that strafing right means positive
        double strafeRevs = Math.abs(strafeInches / CIRCUMFERENCE_INCHES);
        if (strafeInches >= 0) {
            telemetry.addData("Strafing towards right by ", "%.3f inches", strafeInches);

            drive(SPEED,
                    1 * strafeRevs,
                    -1 * strafeRevs,
                    -1 * strafeRevs,
                    1 * strafeRevs);
        } else {
            telemetry.addData("Strafing towards Left by ", "%.3f inches", Math.abs(strafeInches));

            drive(SPEED,
                    -1 * strafeRevs,
                    1 * strafeRevs,
                    1 * strafeRevs,
                    -1 * strafeRevs);
        }
    }

    private void moveStraightLine(double movementInInches) {
        double moveInRevs = movementInInches / CIRCUMFERENCE_INCHES;
        telemetry.addData("Moving ", "%.3f inches", movementInInches);
        telemetry.update();
        drive(SPEED, moveInRevs, moveInRevs, moveInRevs, moveInRevs);
    }

    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs, double rightBackRevs) {

        int LFdrivetarget = (int) (leftFrontRevs * MOTOR_TICK_COUNTS) + leftFrontDrive.getCurrentPosition();
        int LBdrivetarget = (int) (leftBackRevs * MOTOR_TICK_COUNTS) + leftBackDrive.getCurrentPosition();
        int RFdrivetarget = (int) (rightFrontRevs * MOTOR_TICK_COUNTS) + rightFrontDrive.getCurrentPosition();
        int RBdrivetarget = (int) (rightBackRevs * MOTOR_TICK_COUNTS) +  rightBackDrive.getCurrentPosition();

        leftFrontDrive.setTargetPosition(LFdrivetarget);
        leftBackDrive.setTargetPosition(LBdrivetarget);
        rightFrontDrive.setTargetPosition(RFdrivetarget);
        rightBackDrive.setTargetPosition(RBdrivetarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
//            telemetry.addLine("Current Position of the Motors")
//                    .addData("Left Front  ", "%d", leftFrontDrive.getCurrentPosition())
//                    .addData("Left Back ", "%d", leftBackDrive.getCurrentPosition())
//                    .addData("Right Front ", "%d", rightFrontDrive.getCurrentPosition())
//                    .addData("Right Back ", "%df", rightBackDrive.getCurrentPosition());
//
//            telemetry.addLine("Target Positions of the Motors")
//                    .addData("Left Front  ", "%d", LFdrivetarget)
//                    .addData("Left Back ", "%d", LBdrivetarget)
//                    .addData("Right Front ", "%d", RFdrivetarget)
//                    .addData("Right Back ", "%df", RBdrivetarget);

            //telemetry.update();
        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);


        sleep(50);
        // sleep(250);
    }

}

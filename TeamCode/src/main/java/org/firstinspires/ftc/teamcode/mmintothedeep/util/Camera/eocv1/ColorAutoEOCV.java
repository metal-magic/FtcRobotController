package org.firstinspires.ftc.teamcode.mmintothedeep.util.Camera.eocv1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.concurrent.TimeUnit;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.RotatedRect;
import org.opencv.core.Rect;


import org.opencv.core.Point;

import org.opencv.core.Mat;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;



/*
 * This OpMode determines the best Exposure for minimizing image motion-blur on a Webcam
 * Note that it is not possible to control the exposure for a Phone Camera, so if you are using a Phone for the Robot Controller
 * this OpMode/Feature only applies to an externally connected Webcam
 *
 * The goal is to determine the smallest (shortest) Exposure value that still provides reliable Tag Detection.
 * Starting with the minimum Exposure and maximum Gain, the exposure is slowly increased until the Tag is
 * detected reliably from the likely operational distance.
 *
 *
 * The best way to run this optimization is to view the camera preview screen while changing the exposure and gains.
 *
 * To do this, you need to view the RobotController screen directly (not from Driver Station)
 * This can be done directly from a RC phone screen (if you are using an external Webcam), but for a Control Hub you must either plug an
 * HDMI monitor into the Control Hub HDMI port, or use an external viewer program like ScrCpy (https://scrcpy.org/)
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

@Autonomous//(name="Align camera to colored sample", group = "Concept")
// @Disabled
public class ColorAutoEOCV extends LinearOpMode{
    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private ColorBlobLocatorProcessor blobProcessor = null;

    /* Declare all motors as null */
    Date currentTime = new Date();

    private boolean USE_WEBCAM = true;
    private boolean REFRESH_WEBCAM = false;

    @Override public void runOpMode()
    {

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -1))
                //.setDrawContours(true) // (DO NOT UNCOMMENT)                       // Show contours on the Stream Preview
                .setBlurSize(6)                               // Smooth the transitions between different colors in image
                .setErodeSize(5)
                .setDilateSize(2)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "C:/Nandu/EOCV/WIN_20241112_17_14_12_Pro.jpg"))
                .build();

        /*ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.6875, 0.7083, 0.6875, -0.7083))  // search central 1/4 of camera view
                // .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                //.setErodeSize(6)
                //.setDilateSize(6)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .build();*/


        // Establish Min and Max Gains and Exposure.  Then set a low exposure with high gain
        getCameraSetting();
        // myExposure = Math.min(5, minExposure);

        // Wait for the match to begin.
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();


        waitForStart();

        while (opModeIsActive())
        {
//            telemetry.addLine("Find lowest Exposure that gives reliable detection.");
//            telemetry.addLine("Use Left bump/trig to adjust Exposure.");
//            telemetry.addLine("Use Right bump/trig to adjust Gain.\n");

            // Display how many Tags Detected
            /*
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            int numTags = currentDetections.size();
            if (numTags > 0 )
                telemetry.addData("Tag", "####### %d Detected  ######", currentDetections.size());
            else
                telemetry.addData("Tag", "----------- none - ----------");
            */
//            telemetry.addData("Exposure","%d  (%d - %d)", myExposure);
//            telemetry.addData("Gain","%d  (%d - %d)", myGain);
//            telemetry.addData("DS preview on/off", "3 dots, Camera Stream\n");

            // Request the most recent color analysis.
            // This will return the closest matching colorSwatch and the predominant RGB color.
            // Note: to take actions based on the detected color, simply use the colorSwatch in a comparison or switch.
            //  eg:
            //      if (result.closestSwatch == PredominantColorProcessor.Swatch.RED) {... some code  ...}

            // PredominantColorProcessor.Result result = colorLocator.getAnalysis();

            // Display the Color Sensor result.
            //  telemetry.addData("Best Match:", result.closestSwatch);
            //  telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
            //  telemetry.update();

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(500, 500000, blobs);

            sleep(200);

//            for(ColorBlobLocatorProcessor.Blob b : blobs) {
//                RotatedRect boxFit = b.getBoxFit();
//                telemetry.addLine(String.valueOf(b.getContourArea()));
//                telemetry.addLine(String.valueOf(b.getDensity()));
//                telemetry.addLine(String.valueOf(b.getAspectRatio()));
//                telemetry.addLine(String.valueOf((int) boxFit.center.x));
//                telemetry.addLine(String.valueOf((int) boxFit.center.y));
//                if (Math.abs(((int) boxFit.center.x) - 320) <= 20) {
//                    telemetry.addLine("CENTERED");
//                }
//                if ((int) boxFit.center.x < 280) {
//                    strafe(-1);
//                }
//                else if ((int) boxFit.center.x > 360) {
//                    strafe(1);
//                }
//            }

            org.opencv.core.Size myBoxFitSize;
            if (!blobs.isEmpty()) {
                ColorBlobLocatorProcessor.Blob b = blobs.get(0);
                RotatedRect boxFit = b.getBoxFit();
                myBoxFitSize = boxFit.size;
                double boxWidth = myBoxFitSize.width;
                double boxHeight = myBoxFitSize.height;
                int currX = (int) boxFit.center.x;
                double error = 320 - currX;
                int angle = (int) boxFit.angle;
                telemetry.addLine(String.valueOf((int) boxFit.center.x));
                telemetry.addLine(String.valueOf(18644/Math.min(boxHeight, boxWidth)));
                Point[] myBoxCorners = new Point[4];
                boxFit.points(myBoxCorners);
                // this points() method does not return values, it populates the argument
                for (int i = 0; i < 4; i++)
                {
                    telemetry.addLine(String.format("boxFit corner %d (%d,%d)",
                            i, (int) myBoxCorners[i].x, (int) myBoxCorners[i].y));
                }
                // Display getContourPoints(), an array of the contour's many (X, Y) vertices
                Point[] myContourPoints;
                myContourPoints = b.getContourPoints();

                // leastRegLine(splitX(myContourPoints), splitY(myContourPoints));
                Object[] endPointArray = groupLSRL(myContourPoints);
                for (int i=0; i<endPointArray.length; i+=2) {
                    telemetry.addLine("Starting Point: " + ((Point) endPointArray[i]).x + ", " + (480-((Point) endPointArray[i]).y));
                    telemetry.addLine("Ending Point: " + ((Point) endPointArray[i+1]).x + ", " + (480-((Point) endPointArray[i+1]).y));
                }
                telemetry.addLine("TEST");
                telemetry.addLine(String.valueOf(endPointArray.length));
                int j = 0;
                for(Point thisContourPoint : myContourPoints)
                {
//                    telemetry.addLine(String.format("contour vertex %d (%d,%d)",
//                            j, (int) thisContourPoint.x, (int) thisContourPoint.y));
//                    j += 1;
                    j+=1;
                    telemetry.addLine(j+"(" + (int) thisContourPoint.x + "," + (int) thisContourPoint.y + "),");
                }
            }





            sleep(20);
            telemetry.update();
        }
    }

    /**
     * Initialize the ColorSensor processor
     */

    public void initColorProcessor() {
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.2, 0.2, 0.2, -0.2))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .build();
    }

    /**
     * Initialize the ColorLocator processor
     */

    public void initColorBlobsProcessor(ColorRange color) {
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0, 0.5, -1))  // search central 1/4 of camera view
                // .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                //.setErodeSize(6)
                //.setDilateSize(6)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .build();
    }

//    public void autoAlign() {
//
//
//    }


    /*
        Manually set the camera gain and exposure.
        Can only be called AFTER calling initAprilTag();
        Returns true if controls are set.
     */

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

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }


    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Rect rect = new Rect(20, 20, 50, 50);

        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
    }

    // Splits Point[] into X coordinate array and Y coordinate array for simple processing

    public int[] splitX(Point[] array) {
        int[] xArr = new int[array.length];
        for (int i=0; i<xArr.length; i++) {
            xArr[i] = (int) array[i].x;
        }
        return xArr;
    }

    public int[] splitX(ArrayList<Point> array) {
        int[] xArr = new int[array.size()];
        for (int i=0; i<xArr.length; i++) {
            xArr[i] = (int) array.get(i).x;
        }
        return xArr;
    }

    public int[] splitY(Point[] array) {
        int[] yArr = new int[array.length];
        for (int j=0; j<yArr.length; j++) {
            yArr[j] = 480 - (int) array[j].y; // Converts from Computer Vision coordinates (Y increases from top of the image) to mathematical coordinates (Y increases from bottom of the image)
        }
        return yArr;
    }

    public int[] splitY(ArrayList<Point> array) {
        int[] yArr = new int[array.size()];
        for (int j=0; j<yArr.length; j++) {
            yArr[j] = 480 - (int) array.get(j).y; // Converts from Computer Vision coordinates (Y increases from top of the image) to mathematical coordinates (Y increases from bottom of the image)
        }
        return yArr;
    }

    private double calculateB(
            int[] x, int[] y)
    {
        int n = x.length;

        // sum of array x
        int sx = Arrays.stream(x).sum();

        // sum of array y
        int sy = Arrays.stream(y).sum();

        // for sum of product of x and y
        int sxsy = 0;

        // for sum of square of x
        int sx2 = 0;

        // sum of product of x and y
        // sum of square of x
        for (int i = 0; i < n; i++) {
            sxsy += x[i] * y[i];
            sx2 += x[i] * x[i];
        }
        double b = (double)(n * sxsy - sx * sy)
                / (n * sx2 - sx * sx);

        return b;
    }

    private double calculateR(int[] x, int[] y)
    {
        int n = x.length;

        // sum of array x
        int sx = Arrays.stream(x).sum();

        // sum of array y
        int sy = Arrays.stream(y).sum();

        // for sum of product of x and y
        int sxsy = 0;

        // for sum of square of x
        int sx2 = 0;

        // for sum of square of y
        int sy2 = 0;

        // sum of product of x and y
        // sum of square of x
        for (int i = 0; i < n; i++) {
            sxsy += x[i] * y[i];
            sx2 += x[i] * x[i];
            sy2 += y[i] * y[i];
        }
        double r = (double)(n*sxsy-sx*sy)/Math.sqrt((n*sx2-sx*sx)*(n*sy2-sy*sy));

        return r*r;
    }

    // Function to find the
    // least regression line
    public double[] leastRegLine(int X[], int Y[])
    {

        // Finding b
        double b = calculateB(X, Y);

        int n = X.length;
        int meanX = Arrays.stream(X).sum() / n;
        int meanY = Arrays.stream(Y).sum() / n;

        // calculating a
        double a = meanY - b * meanX;

        // Printing regression line
        // telemetry.addLine("y = " + String.valueOf(a) + " + " + String.valueOf(b) + "*X");

        double[] output = new double[3];
        output[0] = a;
        output[1] = b;
        output[2] = calculateR(X, Y);
        return output;
    }


    // Groups pointArray into various groups of points based on LSRL thresholds
    public Object[] groupLSRL(Point[] pointArray) {
        ArrayList<Point> singleGroupPointArray = new ArrayList<Point>(); // ArrayList of points - Single Group (Based on LSRL)
        ArrayList<Integer> totalGroupArray = new ArrayList<Integer>(); // ArrayList of groups
        ArrayList<Point> endPointArrayList = new ArrayList<Point>(); // Calculates endpoints of each regression segment - Odds are startpoints and Evens are endpoints

        for (int i=0; i < pointArray.length; i++) {
            telemetry.addLine("(" + pointArray[i].x + "," + pointArray[i].y + "),");
            singleGroupPointArray.add(pointArray[i]);
            if (singleGroupPointArray.size() >= 5) {
                double[] inputR = leastRegLine(splitX(singleGroupPointArray), splitY(singleGroupPointArray));
                double threshold = 1 - ((double) 2.7128 / (singleGroupPointArray.size() + 2.7128));
                if (inputR[2] < threshold) {
                    singleGroupPointArray.remove(singleGroupPointArray.size() - 1);
                    totalGroupArray.add(singleGroupPointArray.size());
                    endPointArrayList.add(singleGroupPointArray.get(0));
                    endPointArrayList.add(singleGroupPointArray.get(singleGroupPointArray.size()-1));
                    singleGroupPointArray.clear();
                }
            }
            // telemetry.addLine(String.valueOf(totalGroupArray.get(totalGroupArray.size() - 1)));
        }
        for (int j=0; j < totalGroupArray.size(); j++) {
            telemetry.addLine(String.valueOf(totalGroupArray.get(j)));
        }
        Object[] endPointArray = endPointArrayList.toArray();
        return endPointArray;
    }
}

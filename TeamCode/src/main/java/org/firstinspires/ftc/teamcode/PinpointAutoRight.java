package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;
import java.util.Locale;

@Autonomous(name="localizeWithTag", group="Pinpoint")
//@Disabled

public class PinpointAutoRight extends LinearOpMode {

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;

    VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    static final double cameraOffsetY = -127.0;
    static final double cameraOffsetX = -203.2;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    //int[] viewIds = VisionPortal.makeMultiPortalView(1, VisionPortal.MultiPortalLayout.VERTICAL); // used for multiple vision portals, but we have only 1 for now

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5
    }

    // THE POSITIONS THE ROBOT WILL DRIVE TO IN AUTO
    static Pose2D startingPos = new Pose2D(DistanceUnit.MM, 1600, 460, AngleUnit.DEGREES, 175); // Starting position
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM, 900, 260, AngleUnit.DEGREES, 175); // High Chamber
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 1130, 1077, AngleUnit.DEGREES, 90); // AprilTag
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, 327, 1400, AngleUnit.DEGREES, 90); // Ready to push
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, 1400, 1270, AngleUnit.DEGREES, 0); // HP Area
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, 1534, 1330, AngleUnit.DEGREES, 0);

    @Override
    public void runOpMode() {
        // SPLIT UP INTO INIT METHODS TO CLEAN UP MAIN METHOD

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        initPortal(); // initialize the apriltag portal
        initMotor(); // initialize the drive wheel motors
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0, -180.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();


        odo.setPosition(startingPos);
        odo.update();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        odoTelemetry(); // initialized telemetry for the odo + tells us it is initialized

        boolean atTarget = false; // boolean to determine whether the robot is at stationary position or not. if so, then all the wheels will have 0 power

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            odo.update();

            // localizing with aprilTags whenever it sees an apriltag
//            if (!tagProcessor.getDetections().isEmpty()) {
//
//                // tag next to observation zone for specimen
//                if (tagProcessor.getDetections().get(0).id == 11 || tagProcessor.getDetections().get(0).id == 14) {
//
//                    if (!tagProcessor.getDetections().isEmpty()) { // checking once again just to be safe ... else it throws an error
//                        // saving the x, y, and heading into variables for ease of access
//                        double X = tagProcessor.getDetections().get(0).ftcPose.x;
//                        double Y = tagProcessor.getDetections().get(0).ftcPose.y;
//                        double H = tagProcessor.getDetections().get(0).ftcPose.yaw;
//                        // printing pose onto the driver station
//
//
//                        // localized x, y, and heading = pose from the april tag with 0, 0 in the right corner of field (next to submersible
//                        double lX = (Math.cos(Math.toRadians(H)) * X) + 24; // aprilTag is 24 inches from the size of the game field
//                        lX -= 5; // 5 for offset from camera to middle
//                        double lY = (Math.cos(Math.toRadians(H)) * Y);
//                        double lH = -90 + H; // heading offset by 90 degrees vs odometry
//
//                        telemetry.addData("aprilX", lX);
//                        telemetry.addData("aprilY", lY);
//                        telemetry.addData("aprilY", lH);
//
//                        // swap y and x because of how it is on pinpoint
//                        startingPos = new Pose2D(DistanceUnit.INCH, lX, lY, AngleUnit.DEGREES, lH);
//
//                        // set this position as the odometry position
//                        odo.setPosition(startingPos);
//                    }
//                }
//            }

            switch (stateMachine) {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    odo.setPosition(startingPos);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;
                case DRIVE_TO_TARGET_1:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), TARGET_1, 0.4, 0.2)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    //drive to the second target
                    if (nav.driveTo(odo.getPosition(), TARGET_2, 0.4, 2)) {
                        telemetry.addLine("at position #2!");
                        if (!tagProcessor.getDetections().isEmpty()) {
                            odo.setPosition(returnAprilTagPose(odo.getPosition()));
                        }
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;

                case DRIVE_TO_TARGET_3:
                    if (nav.driveTo(odo.getPosition(), TARGET_3, 0.4, 0.2)) {
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                        long t = System.currentTimeMillis();
                        long endTimer = t + 800;
                        while (System.currentTimeMillis() < endTimer) {
                            leftFrontDrive.setPower(1);
                            leftBackDrive.setPower(1);
                            rightBackDrive.setPower(-1);
                            rightFrontDrive.setPower(-1);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if (nav.driveTo(odo.getPosition(), TARGET_4, 0.4, 0.1)) {
                        telemetry.addLine("at position #4");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if (nav.driveTo(odo.getPosition(), TARGET_5, 0.4, 0.2)) {
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;
                case AT_TARGET:
                    atTarget = true;
                    leftFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    break;
            }

            if (!atTarget) {
                //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
                leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            }

            telemetry.addData("current state:", stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetryAprilTag();


            telemetry.update();

        }
    }

    public void initPortal() {

        // Because we want to show two camera feeds simultaneously, we need to inform
        // the SDK that we want it to split the camera monitor area into two smaller
        // areas for us. It will then give us View IDs which we can pass to the
        // individual
        // vision portals to allow them to properly hook into the UI in tandem.

        // We extract the two view IDs from the array to make our lives a little easier
        // later.
        // NB: the array is 2 long because we asked for 3 portals up above.
        //int portal1ViewId = viewIds[0];

        // drawing information on the driver station camera screen
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(513.474, 513.474, 316.919, 249.760)
                .build();

        // stating the webcam
        visionPortal = new VisionPortal.Builder()
                //.setLiveViewContainerId(portal1ViewId)
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "tagCam"))
                .setCameraResolution(new Size(640, 480))
                .build();

    }

    public void initMotor() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    public void odoTelemetry() {

        // initialized telemetry for the odo + tells us it is initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());

        telemetry.addData("x", odo.getPosX());
        telemetry.addData("y", odo.getPosY());

        telemetry.update();
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

// Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

// Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }

    private Pose2D returnAprilTagPose(Pose2D current) {

        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

// Step through the list of detections and change our current position if we see one
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                Pose2D newPose = new Pose2D(DistanceUnit.INCH,
                        detection.robotPose.getPosition().y, -detection.robotPose.getPosition().x, AngleUnit.DEGREES, detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                return newPose;
            }
        }
        return current;
    }

    }




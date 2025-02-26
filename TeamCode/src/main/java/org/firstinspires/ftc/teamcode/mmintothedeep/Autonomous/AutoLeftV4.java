package org.firstinspires.ftc.teamcode.mmintothedeep.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.DriveToPoint;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.mmintothedeep.util.Camera.eocv1.CameraPickup;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Autonomous(name="!!AutoLeftV4Color")
//@Disabled

public class AutoLeftV4 extends LinearOpMode {

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor pivotMotor;
    int newTarget;

    long timeToTransfer5th;

    public static int ratioY = 5;
    public static int ratioX = 3;

    static final double cameraOffsetY = -127.0;
    static final double cameraOffsetX = -203.2;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private ColorBlobLocatorProcessor colorLocator;
    private int myExposure;
    private int myGain;


    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5,
        DRIVE_TO_TARGET_6,
        DRIVE_TO_TARGET_7,
        DRIVE_TO_TARGET_8,
        DRIVE_TO_TARGET_9,
        DRIVE_TO_TARGET_10,
        DRIVE_TO_TARGET_11,
        DRIVE_TO_TARGET_12,
        DRIVE_TO_TARGET_13,
        DRIVE_TO_TARGET_14,
        DRIVE_TO_TARGET_15,
        ALIGN_TO_SAMPLE
    }

//    static final Pose2D startingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // Starting position
//    static final Pose2D BASKET_TARGET = new Pose2D(DistanceUnit.MM,-453,163,AngleUnit.DEGREES,42);
//    static final Pose2D SAMPLE_1 = new Pose2D(DistanceUnit.MM,-302,450,AngleUnit.DEGREES,90);
//    static final Pose2D SAMPLE_2 = new Pose2D(DistanceUnit.MM,-522,450,AngleUnit.DEGREES,90);
//    static final Pose2D SAMPLE_3 = new Pose2D(DistanceUnit.MM,-500,535,AngleUnit.DEGREES,120);




//    static final Pose2D startingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // Starting position
//    static final Pose2D BASKET_TARGET = new Pose2D(DistanceUnit.MM,-448,155,AngleUnit.DEGREES,45);
//    static final Pose2D SAMPLE_1 = new Pose2D(DistanceUnit.MM,-302,450,AngleUnit.DEGREES,90);
//    static final Pose2D SAMPLE_2 = new Pose2D(DistanceUnit.MM,-522,450,AngleUnit.DEGREES,90);
//    static final Pose2D SAMPLE_3 = new Pose2D(DistanceUnit.MM,-500,545,AngleUnit.DEGREES,121);

    static final Pose2D startingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // Starting position
    //    static final Pose2D BASKET_TARGET = new Pose2D(DistanceUnit.MM,-450,110,AngleUnit.DEGREES,40);
//    static final Pose2D BASKET_TARGET = new Pose2D(DistanceUnit.MM,-443,180,AngleUnit.DEGREES,50);

    static final Pose2D BASKET_TARGET = new Pose2D(DistanceUnit.MM,-415,180,AngleUnit.DEGREES,46);


    //    static final Pose2D SAMPLE_1 = new Pose2D(DistanceUnit.MM,-302,450,AngleUnit.DEGREES,90);
    static final Pose2D SAMPLE_1 = new Pose2D(DistanceUnit.MM,-300,390,AngleUnit.DEGREES,90);

    //    static final Pose2D SAMPLE_2 = new Pose2D(DistanceUnit.MM,-522,450,AngleUnit.DEGREES,90);
    static final Pose2D SAMPLE_2 = new Pose2D(DistanceUnit.MM,-530,382,AngleUnit.DEGREES,90);

    //    static final Pose2D SAMPLE_3 = new Pose2D(DistanceUnit.MM,-515,570,AngleUnit.DEGREES,120);
    static final Pose2D SAMPLE_3 = new Pose2D(DistanceUnit.MM,-510,450,AngleUnit.DEGREES,117);

    static final Pose2D WAYPOINT_SUB = new Pose2D(DistanceUnit.MM,-320,1225,AngleUnit.DEGREES,0);

    static final Pose2D SUB = new Pose2D(DistanceUnit.MM,250,1650,AngleUnit.DEGREES,0);

    static Pose2D sample_pos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);


    static final double slidePosDown = UtilityValues.SLIDE_POS_DOWN;
    static final double slidePosSpecDown = 2300; //UtilityValues.SLIDE_POS_SPEC_DOWN;
    static final double slidePosSpecUp = UtilityValues.SLIDE_POS_SPEC_UP;
    static final double slidePosUp = UtilityValues.SLIDE_POS_SAMP;
    static final double slidePosTransfer = UtilityValues.SLIDE_POS_TRANSFER;


    static final double flipPosDown = UtilityValues.FLIP_POS_DOWN;
    static final double flipPosScore = UtilityValues.FLIP_POS_SCORE;

    static final double clipPosClose = UtilityValues.CLIP_POS_CLOSE;
    static final double clipPosOpen = UtilityValues.CLIP_POS_OPEN;

    static final double pivotPosDown = UtilityValues.PIVOT_POS_DOWN;
    static final double pivotPosHover = UtilityValues.PIVOT_POS_HOVER;
    static final double pivotPosFloat = UtilityValues.PIVOT_POS_FLOAT;
    static final double pivotPosTransfer = UtilityValues.PIVOT_POS_TRANSFER;

    boolean slideUp = false;
    boolean slideDown = false;

    // public Servo gripperServo2 = null;
    public Servo clipServo = null;
    public Servo flipServo = null;
    //public Servo pivotServo = null;
    public DcMotor linearSlideMotor = null;
    public Servo gripperServo1 = null;

    boolean atTarget = false;
    public Servo turnServo = null;

    public static final int toleranceMotorSleep = 20;


    @Override
    public void runOpMode() {

//        initPortal();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        clipServo = hardwareMap.servo.get("clipServo");

        flipServo = hardwareMap.servo.get("flipServo");

        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        turnServo = hardwareMap.servo.get("turnServo");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setDirection(UtilityValues.compLeftBackDirection);
        leftFrontDrive.setDirection(UtilityValues.compLeftFrontDirection);
        rightBackDrive.setDirection(UtilityValues.compRightBackDirection);
        rightFrontDrive.setDirection(UtilityValues.compRightFrontDirection);

        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(0, 130); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.addData("Xpos", odo.getEncoderX());
        telemetry.addData("Ypos", odo.getEncoderY());
        telemetry.update();

        clipServo.setPosition(clipPosClose);
        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        initColorBlobsProcessor(ColorRange.YELLOW);

        getCameraSetting();
        myExposure = 30;
        myGain = 230;
        setManualExposure(myExposure, myGain);

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            switch (stateMachine){
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    odo.setPosition(startingPos);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_DOWN_AUTO, 0.3);
                    sleep(200);
                    break;
                case DRIVE_TO_TARGET_1:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), BASKET_TARGET, 0.75, 0.5)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        pivotMotor.setPower(0);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        waitSlide((int) slidePosUp);
                        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_ALIGN, 0.3);
                        flipServo.setPosition(flipPosScore);
                        sleep(400);
                        flipServo.setPosition(flipPosDown);
                        sleep(250);
                    } else {
                        runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_SAMP, 1);
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    //drive to the first sample
                    if (nav.driveTo(odo.getPosition(), SAMPLE_1, 0.75, 0.4)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                        // pick up
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        flipServo.setPosition(flipPosDown);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_DOWN, 0.3);

                        waitSlide((int) UtilityValues.SLIDE_POS_TRANSFER);

                        sleep(200);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        sleep(200);
                        // transfer
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER, 0.5);
                        turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);
                        sleep(500);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        sleep(300);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT, 0.4);
                        sleep(200);
                    } else {

                        runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);

                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if(nav.driveTo(odo.getPosition(), BASKET_TARGET, 0.75, 0.4)){
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;

                        waitSlide((int) UtilityValues.SLIDE_POS_SAMP);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_ALIGN, 0.2);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        flipServo.setPosition(flipPosScore);
                        sleep(400);
                        flipServo.setPosition(flipPosDown);
                        sleep(250);
                    } else {
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT, 0.4);

                        runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_SAMP, 1);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_SUB, 0.4);
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    //drive to the second sample
                    if (nav.driveTo(odo.getPosition(), SAMPLE_2, 0.75, 0.4)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        flipServo.setPosition(flipPosDown);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_DOWN, 0.3);

                        waitSlide((int) UtilityValues.SLIDE_POS_TRANSFER);

                        sleep(200);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        sleep(200);
                        // transfer
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER, 0.4);
                        //pivotServo.setPosition(pivotPosTransfer);
                        //sleep(500);
                        turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);
                        sleep(500);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        sleep(300);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_ALIGN, 0.6);
                        sleep(100);
                        //pivotServo.setPosition(pivotPosFloat);
                    } else {


                        runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if(nav.driveTo(odo.getPosition(), BASKET_TARGET, 0.75, 0.4)){
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;

                        waitSlide((int) UtilityValues.SLIDE_POS_SAMP);

                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_ALIGN, 0.2);
                        flipServo.setPosition(flipPosScore);
                        sleep(400);
                        flipServo.setPosition(flipPosDown);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        sleep(250);
                    } else {
                        runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_SAMP, 1);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_SUB, 0.4);
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    //drive to the third sample
                    if (nav.driveTo(odo.getPosition(), SAMPLE_3, 0.75, 0.4)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        flipServo.setPosition(flipPosDown);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_DOWN, 0.3);

                        waitSlide((int) UtilityValues.SLIDE_POS_TRANSFER);

                        sleep(300);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        sleep(500);
                        // transfer
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER, 0.5);
                        //pivotServo.setPosition(pivotPosTransfer);
                        //sleep(500);
                        turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);
                        sleep(400);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        sleep(300);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT, 0.4);
                        //pivotServo.setPosition(pivotPosFloat);
                        sleep(500);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                    } else {
                        runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
                    }
                    break;
                case DRIVE_TO_TARGET_7:
                    if(nav.driveTo(odo.getPosition(), BASKET_TARGET, 0.75, 0.4)){
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;

                        waitSlide((int) UtilityValues.SLIDE_POS_SAMP);

                        flipServo.setPosition(flipPosScore);
                        sleep(400);
                        flipServo.setPosition(flipPosDown);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT, 0.2);
                        sleep(300);
                        atTarget = false;
                    } else {
                        runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_SAMP, 1);

                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    if(nav.driveTo(odo.getPosition(), WAYPOINT_SUB, 0.875, 0.2)){
                        telemetry.addLine("at position #4");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    } else {
                        runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_SUB, 0.8);
                    }
                    break;
                case DRIVE_TO_TARGET_9:
                    if(nav.driveTo(odo.getPosition(), SUB, 0.75, 0.3)) {
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.ALIGN_TO_SAMPLE;
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_ALIGN, 0.5);
                        alignToSample();
                    } else {
                        runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
                    }
                    break;
                case DRIVE_TO_TARGET_10:
                    if(nav.driveTo(odo.getPosition(), BASKET_TARGET, 0.75, 0.4)) {
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.AT_TARGET;

                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_ALIGN, 0.2);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);

                        waitSlide((int) UtilityValues.SLIDE_POS_SAMP);

                        flipServo.setPosition(flipPosScore);
                        sleep(300);
                        flipServo.setPosition(flipPosDown);
                    } else {
                        // events in reverse order for else logic
                        if (timeElapsed5th(1700)) {
                            turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_SAMP, 1);
                        } else if (timeElapsed5th(1200)) {
                            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT, 0.2);
                        } else if (timeElapsed5th(900)) {
                            gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        } else if (timeElapsed5th(500)) {
                            gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER, 0.6);
                            turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);
                        } else {
                            gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        }

                    }
                    break;
                case ALIGN_TO_SAMPLE:
                    if (nav.driveTo(odo.getPosition(), sample_pos, 0.65, 0.4)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_10;
                        //sleep(200);
                        gripperServo1.setPosition(0.35); //UtilityValues.GRIPPER_POS_OPEN);
                        //alignToSample();
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_DOWN, 0.5);
                        sleep(300);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        sleep(400);
                        turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER, 0.5);
                        sleep(100);
                        timeToTransfer5th = System.currentTimeMillis();

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


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            if (!atTarget) {
                leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            } else {
                leftFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                if (linearSlideMotor.getCurrentPosition() > 50) {
                    linearSlideMotor.setPower(-1);
                }
            }

            telemetry.addData("current state:",stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);


            telemetry.update();

        }
    }

    public boolean timeElapsed5th(int elapsed) {

        return System.currentTimeMillis() >= (timeToTransfer5th + elapsed);

    }

    public void runToPosition(DcMotor motor, int ticks, double power) {
        motor.setTargetPosition(ticks);
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//    private void alignToSample () {
//
//        // Read the current list
//        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
//
//        ColorBlobLocatorProcessor.Util.filterByArea(2000, 20000, blobs);  // filter out very small blobs.
//
//        while (blobs.isEmpty()) {
//            blobs = colorLocator.getBlobs();
//        }
//
//        ColorBlobLocatorProcessor.Blob largestBlob = blobs.get(0);
//        RotatedRect boxFit = largestBlob.getBoxFit();
//
//        sample_pos = new Pose2D(DistanceUnit.INCH, odo.getPosX() / 25.4 + (240 - (int) boxFit.center.y) / 240.0 * 4.0, odo.getPosY() / 25.4 + -1 * ((int) boxFit.center.x - 320) / 320.0 * 6.0, AngleUnit.DEGREES, 0);
//
//        telemetry.addData("Pose X", sample_pos.getX(DistanceUnit.INCH));
//        telemetry.addData("Pose Y", sample_pos.getY(DistanceUnit.INCH));
//        telemetry.addData("Pose Heading", sample_pos.getHeading(AngleUnit.DEGREES));
//
//        telemetry.update();
//        sleep(20);
//    }

    public void initColorBlobsProcessor(ColorRange color) {
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1 , 1, -1))  // search central 1/4 of camera view
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

        // Read the current list
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(2000, 70000, blobs);  // filter out very small blobs.

        while (blobs.isEmpty()) {
            blobs = colorLocator.getBlobs();
        }

        double offsetX = 240.0;
        double offsetY = 220.0;

        int index = 0;
        double lowestScore = 1000000;
        int i = 0;

        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            RotatedRect boxFit = b.getBoxFit();
            double currAngle = boxFit.angle;
            if (boxFit.size.width < boxFit.size.height) {
                currAngle -= 90;
            }
            double score = (Math.abs(currAngle+90)*5+Math.sqrt(Math.pow((offsetY-boxFit.center.y), 2)+Math.pow((offsetX-boxFit.center.x), 2)));
            if (score < lowestScore) {
                lowestScore = score;
                index = i;
            }
            i++;
        }

        ColorBlobLocatorProcessor.Blob bestBlob = blobs.get(0);
        RotatedRect boxFit = bestBlob.getBoxFit();

        sample_pos = new Pose2D(DistanceUnit.INCH, odo.getPosX() / 25.4 + (offsetY - (int) boxFit.center.y) / 240.0 * ratioY, odo.getPosY() / 25.4 + (-1) * ((int) boxFit.center.x - offsetX) / 320.0 * ratioX, AngleUnit.DEGREES, 0);

        telemetry.addData("lowestScore", lowestScore);

        telemetry.addData("Pose X", sample_pos.getX(DistanceUnit.INCH));
        telemetry.addData("Pose Y", sample_pos.getY(DistanceUnit.INCH));
        telemetry.addData("Pose Heading", sample_pos.getHeading(AngleUnit.DEGREES));

        telemetry.update();
        sleep(20);
    }

    public void waitSlide(int pos) {

        while (linearSlideMotor.getCurrentPosition() < pos - toleranceMotorSleep || linearSlideMotor.getCurrentPosition() > pos + toleranceMotorSleep) {
            // nothing
        }

    }


}
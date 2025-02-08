package org.firstinspires.ftc.teamcode.mmintothedeep.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.DriveToPoint;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

import java.util.Locale;

@Autonomous(name="!!!STATES - AUTO RIGHT 3+0+PARK using claw")
//@Disabled

public class AutoRightClaw extends LinearOpMode {

    int newTarget;

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            6, -3.5, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    static final double cameraOffsetY = -127.0;
    static final double cameraOffsetX = -203.2;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

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
        DRIVE_TO_TARGET_16,
        DRIVE_TO_TARGET_17,
        DRIVE_TO_TARGET_18,
        DRIVE_TO_TARGET_19,
        DRIVE_TO_TARGET_20,
        DRIVE_TO_TARGET_21,
        DRIVE_TO_TARGET_22,
        DRIVE_TO_TARGET_23,
        DRIVE_TO_TARGET_24,
        DRIVE_TO_TARGET_25
    }

    static final Pose2D startingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // Starting position
    static final Pose2D HIGH_CHAMBER = new Pose2D(DistanceUnit.MM,-300,755,AngleUnit.DEGREES,-90); // Specimen Chamber 1
    static final Pose2D WAYPOINT_1 = new Pose2D(DistanceUnit.MM, 540, 620, AngleUnit.DEGREES, -90); // Specimen Chamber 2
    static final Pose2D WAYPOINT_2 = new Pose2D(DistanceUnit.MM,650,1410, AngleUnit.DEGREES,-90); // April Tag scanning
    static final Pose2D READY_TO_PUSH_1 = new Pose2D(DistanceUnit.MM, 840, 1200, AngleUnit.DEGREES, -90); // April Tag Position
    static final Pose2D PUSH_1 = new Pose2D(DistanceUnit.MM, 840, 210, AngleUnit.DEGREES, -90);
    static final Pose2D WAYPOINT_3 = new Pose2D(DistanceUnit.MM, 840, 1200, AngleUnit.DEGREES, -90);
    static final Pose2D READY_TO_PUSH_2 = new Pose2D(DistanceUnit.MM, 1120, 1200, AngleUnit.DEGREES, -90);
    static final Pose2D PUSH_2 = new Pose2D(DistanceUnit.MM, 1120, 280, AngleUnit.DEGREES, -90);
    static final Pose2D GRAB_WAYPOINT = new Pose2D(DistanceUnit.MM, 880, 310, AngleUnit.DEGREES, 90);
    static final Pose2D GRAB = new Pose2D(DistanceUnit.MM, 900, 50, AngleUnit.DEGREES, 90);
    static final Pose2D WAYPOINT_CHAMBER = new Pose2D(DistanceUnit.MM, -400, 510, AngleUnit.DEGREES, -90);
    static final Pose2D HIGH_CHAMBER_2 = new Pose2D(DistanceUnit.MM, -400, 793, AngleUnit.DEGREES, -90);
    static final Pose2D TRANSFER_FIRST = new Pose2D(DistanceUnit.MM, 979, 393, AngleUnit.DEGREES, 90);
    static final Pose2D TRANSFER_SECOND = new Pose2D(DistanceUnit.MM, 1230, 410, AngleUnit.DEGREES, 90);


    static final double slidePosDown = UtilityValues.SLIDE_POS_DOWN;
    static final double slidePosSpecDown = UtilityValues.SLIDE_POS_SPEC_DOWN; //UtilityValues.SLIDE_POS_SPEC_DOWN;
    static final double slidePosSpecUp = UtilityValues.SLIDE_POS_SPEC_UP;
    static final double slidePosUp = UtilityValues.SLIDE_POS_SAMP;
    static final double slidePosStable = UtilityValues.SLIDE_POS_STABLE;

    static final double flipPosDown = UtilityValues.FLIP_POS_DOWN;
    static final double flipPosScore = UtilityValues.FLIP_POS_SCORE;
    static final double flipPosMid = UtilityValues.FLIP_POS_MID;


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
    public DcMotor pivotMotor = null;
    public DcMotor linearSlideMotor = null;
    public Servo gripperServo1 = null;

    boolean atTarget = false;
    public Servo turnServo = null;

    public boolean condition = false;

    public Pose2D currPose;

    public Pose2D lastPose;

    public boolean third = false;


    @Override
    public void runOpMode() {

//        initPortal();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        clipServo = hardwareMap.servo.get("clipServo");

        flipServo = hardwareMap.servo.get("flipServo");

        gripperServo1 = hardwareMap.servo.get("gripperServo1");

        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
        gripperServo1.setPosition(0.55);

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT_AUTO, 0.4);

        nav.setXYCoefficients(0.008, 0.00001, 20, DistanceUnit.MM, 30);

        while (opModeIsActive()) {
            odo.update();

            switch (stateMachine){
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    odo.setPosition(startingPos);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    clipServo.setPosition(UtilityValues.CLIP_POS_CLOSE);
                    break;
                case DRIVE_TO_TARGET_1:
                    if (nav.driveTo(odo.getPosition(), HIGH_CHAMBER, 0.7, 0.2)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                        while (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(0.9);
                        }
                        while (linearSlideMotor.getCurrentPosition() > UtilityValues.SLIDE_POS_SPEC_DOWN) {
                            linearSlideMotor.setPower(-0.60);
                        }
                        linearSlideMotor.setPower(0);
                        clipServo.setPosition(clipPosOpen);
                        sleep(50);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    if (nav.driveTo(odo.getPosition(), WAYPOINT_1, 0.9, 0.2)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if (nav.driveTo(odo.getPosition(), TRANSFER_FIRST, 0.65, 0.3)){
                        leftFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        telemetry.addLine("at position #3!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        while (linearSlideMotor.getCurrentPosition() > 300) {
                            linearSlideMotor.setPower(-1);
                        }
                        while (linearSlideMotor.getCurrentPosition() < 300) {
                            linearSlideMotor.setPower(0.3);
                        }
                        linearSlideMotor.setPower(0);
                        // pick up
                        //gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        flipServo.setPosition(flipPosDown);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_DOWN_AUTO, 0.3);
                        sleep(300);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        sleep(250);
                        // transfer
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER_AUTO, 0.20);
                        sleep(200);
                        turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);
                        sleep(1200);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        sleep(300);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_SUB_AUTO, 0.4);
                        sleep(300);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        flipServo.setPosition(flipPosMid);
                        sleep(50);
                        flipServo.setPosition(flipPosScore);
                        sleep(300);
                        flipServo.setPosition(flipPosDown);
                    } else {
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_SUB_AUTO, 0.2);
                        if (linearSlideMotor.getCurrentPosition() > 300) {
                            linearSlideMotor.setPower(-1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if (nav.driveTo(odo.getPosition(), TRANSFER_SECOND, 0.65, 0.3)) {
                        leftFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        while (linearSlideMotor.getCurrentPosition() > 300) {
                            linearSlideMotor.setPower(-1);
                        }
                        while (linearSlideMotor.getCurrentPosition() < 300) {
                            linearSlideMotor.setPower(0.3);
                        }
                        linearSlideMotor.setPower(0);
                        // pick up
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        flipServo.setPosition(flipPosDown);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_DOWN_AUTO, 0.3);
                        sleep(300);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        sleep(300);
                        // transfer
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER_AUTO, 0.20);
                        turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);
                        sleep(1200);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        sleep(100);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT_AUTO, 0.5);
                        sleep(350);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        flipServo.setPosition(flipPosMid);
                        sleep(50);
                        flipServo.setPosition(flipPosScore);
                        sleep(350);
                        flipServo.setPosition(flipPosDown);
                        while (linearSlideMotor.getCurrentPosition() > 20) {
                            linearSlideMotor.setPower(-1);
                        }
                        linearSlideMotor.setPower(0);
                    } else {
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_SUB_AUTO, 0.2);
                        if (linearSlideMotor.getCurrentPosition() > 300) {
                            linearSlideMotor.setPower(-1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if (nav.driveTo(odo.getPosition(), PUSH_1, 0.7, 0.2)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if (nav.driveTo(odo.getPosition(), WAYPOINT_3, 0.7, 0.2)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;
                    }
                    break;
                case DRIVE_TO_TARGET_7:
                    if (nav.driveTo(odo.getPosition(), READY_TO_PUSH_2, 0.7, 0.2)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;
                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    if (nav.driveTo(odo.getPosition(), PUSH_2, 0.7, 0.2)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    }
                    break;
                case DRIVE_TO_TARGET_9:
                    if (nav.driveTo(odo.getPosition(), GRAB_WAYPOINT, 0.625, 0.25)){
                        while (linearSlideMotor.getCurrentPosition() > 50) {
                            linearSlideMotor.setPower(-1);
                        }
                        linearSlideMotor.setPower(0);
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_10;
                        clipServo.setPosition(clipPosOpen);
                    }
                    runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT_AUTO, 0.5);
                    break;
                case DRIVE_TO_TARGET_10:
                    if (nav.driveTo(odo.getPosition(), GRAB, 0.625, 0.7)){
                        clipServo.setPosition(clipPosClose);
                        sleep(200);
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_11;
                    }
                    break;
                case DRIVE_TO_TARGET_11:
                    if (nav.driveTo(odo.getPosition(), WAYPOINT_CHAMBER, 0.9, 0.2)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_12;
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_12:
                    if (nav.driveTo(odo.getPosition(), HIGH_CHAMBER_2, 0.725, 0.2)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_13;
                        while (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(1);
                        }
                        while (linearSlideMotor.getCurrentPosition() > UtilityValues.SLIDE_POS_SPEC_DOWN) {
                            linearSlideMotor.setPower(-0.85);
                        }
                        linearSlideMotor.setPower(0);
                        clipServo.setPosition(clipPosOpen);
                        sleep(200);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_13:
                    if (nav.driveTo(odo.getPosition(), GRAB_WAYPOINT, 0.825, 0.2)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_14;
                        while (linearSlideMotor.getCurrentPosition() > 50) {
                            linearSlideMotor.setPower(-1);
                        }
                        linearSlideMotor.setPower(0);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() > 20) {
                            linearSlideMotor.setPower(-1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_14:
                    if (nav.driveTo(odo.getPosition(), GRAB, 0.625, 0.7)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_15;
                        clipServo.setPosition(clipPosClose);
                        sleep(200);
                    }
                    break;
                case DRIVE_TO_TARGET_15:
                    if (nav.driveTo(odo.getPosition(), WAYPOINT_CHAMBER, 0.925, 0.2)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_16;
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_16:
                    if (nav.driveTo(odo.getPosition(), HIGH_CHAMBER_2, 0.825, 0.25)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_17;
                        while (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(0.9);
                        }
                        while (linearSlideMotor.getCurrentPosition() > UtilityValues.SLIDE_POS_SPEC_DOWN) {
                            linearSlideMotor.setPower(-0.85);
                        }
                        linearSlideMotor.setPower(0);
                        clipServo.setPosition(clipPosOpen);
                        sleep(200);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_17:
                    if (nav.driveTo(odo.getPosition(), GRAB_WAYPOINT, 0.675, 0.25)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_18;
                        while (linearSlideMotor.getCurrentPosition() > 50) {
                            linearSlideMotor.setPower(-1);
                        }
                        linearSlideMotor.setPower(0);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() > 20) {
                            linearSlideMotor.setPower(-1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_18:
                    if (nav.driveTo(odo.getPosition(), GRAB, 0.625, 0.25)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_19;
                        clipServo.setPosition(clipPosClose);
                    }
                    break;
                case DRIVE_TO_TARGET_19:
                    if (nav.driveTo(odo.getPosition(), WAYPOINT_CHAMBER, 0.925, 0.2)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_20;
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_20:
                    if (nav.driveTo(odo.getPosition(), HIGH_CHAMBER_2, 0.725, 0.25)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_21;
                        while (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(0.9);
                        }
                        while (linearSlideMotor.getCurrentPosition() > UtilityValues.SLIDE_POS_SPEC_DOWN) {
                            linearSlideMotor.setPower(-0.85);
                        }
                        linearSlideMotor.setPower(0);
                        clipServo.setPosition(clipPosOpen);
                        sleep(200);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_21:
                    if (nav.driveTo(odo.getPosition(), GRAB_WAYPOINT, 0.925, 0.25)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.AT_TARGET;
                    } else {
                        if (linearSlideMotor.getCurrentPosition() > 50) {
                            linearSlideMotor.setPower(-1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
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
            }

            telemetry.addData("current state:",stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("Slide", linearSlideMotor.getCurrentPosition());

            telemetry.update();

        }
    }

    public void runToPosition(DcMotor motor, int ticks, double power) {
        newTarget = ticks;
        motor.setTargetPosition(newTarget);
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void goToZero(DcMotor motor) {
        newTarget = 0;
        motor.setTargetPosition(newTarget);
        motor.setPower(0.8);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getDistance(Pose2D currentPosition, Pose2D targetPosition) {
        double xDistance = Math.abs(currentPosition.getX(DistanceUnit.MM) - targetPosition.getX(DistanceUnit.MM));
        double yDistance = Math.abs(currentPosition.getY(DistanceUnit.MM) - targetPosition.getY(DistanceUnit.MM));
        return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
    }

    public double getHeadingDiff(Pose2D currentPosition, Pose2D targetPosition) {
        return Math.abs(currentPosition.getHeading(AngleUnit.DEGREES) - targetPosition.getHeading(AngleUnit.DEGREES));
    }

    public double getNetPower(Pose2D currrentPosition, Pose2D targetPosition) {
        double distance = getDistance(currrentPosition, targetPosition);
        double heading = getHeadingDiff(currrentPosition, targetPosition);

        double headingPower = heading / 180;
        double distancePower = distance/500;
        double totalPower = headingPower - distancePower;
        if (totalPower > 1) {
            totalPower = 1;
        }
        if (totalPower < 0.2) {
            totalPower = 0.2;
        }

        return  totalPower;

    }

}
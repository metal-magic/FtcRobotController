package org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.competition;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.DriveToPoint;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

@Autonomous(name="!!Competition Auto Left", group="Pinpoint")
//@Disabled

public class CompetitionAutoLeft extends LinearOpMode {

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
        DRIVE_TO_TARGET_15
    }

//    static final Pose2D startingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // Starting position
//    static final Pose2D BASKET_TARGET = new Pose2D(DistanceUnit.MM,-453,163,AngleUnit.DEGREES,42);
//    static final Pose2D SAMPLE_1 = new Pose2D(DistanceUnit.MM,-302,450,AngleUnit.DEGREES,90);
//    static final Pose2D SAMPLE_2 = new Pose2D(DistanceUnit.MM,-522,450,AngleUnit.DEGREES,90);
//    static final Pose2D SAMPLE_3 = new Pose2D(DistanceUnit.MM,-500,535,AngleUnit.DEGREES,120);


    static final Pose2D startingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // Starting position
    static final Pose2D BASKET_TARGET = new Pose2D(DistanceUnit.MM,-440,155,AngleUnit.DEGREES,45);
    static final Pose2D SAMPLE_1 = new Pose2D(DistanceUnit.MM,-302,450,AngleUnit.DEGREES,90);
    static final Pose2D SAMPLE_2 = new Pose2D(DistanceUnit.MM,-522,450,AngleUnit.DEGREES,90);
    static final Pose2D SAMPLE_3 = new Pose2D(DistanceUnit.MM,-500,545,AngleUnit.DEGREES,121);


    static final double slidePosDown = UtilityValues.SLIDE_POS_DOWN;
    static final double slidePosSpecDown = 2300; //UtilityValues.SLIDE_POS_SPEC_DOWN;
    static final double slidePosSpecUp = UtilityValues.SLIDE_POS_SPEC_UP;
    static final double slidePosUp = UtilityValues.SLIDE_POS_SAMP;

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
    public Servo pivotServo = null;
    public DcMotor linearSlideMotor = null;
    public Servo gripperServo1 = null;

    boolean atTarget = false;
    public Servo turnServo = null;


    @Override
    public void runOpMode() {

//        initPortal();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        clipServo = hardwareMap.servo.get("clipServo");

        flipServo = hardwareMap.servo.get("flipServo");

        gripperServo1 = hardwareMap.servo.get("gripperServo1");

        pivotServo = hardwareMap.servo.get("pivotServo");
        pivotServo.setDirection(Servo.Direction.REVERSE);
        pivotServo.setPosition(pivotPosFloat);
        turnServo = hardwareMap.servo.get("turnServo");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

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

        while (opModeIsActive()) {
            odo.update();

            switch (stateMachine){
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
                    if (nav.driveTo(odo.getPosition(), BASKET_TARGET, 0.4, 0.5)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                        while (linearSlideMotor.getCurrentPosition() < slidePosUp) {
                            linearSlideMotor.setPower(1);
                        }
                        linearSlideMotor.setPower(0);
                        flipServo.setPosition(flipPosScore);
                        sleep(400);
                        flipServo.setPosition(flipPosDown);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < slidePosUp) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    //drive to the second target
                    if (nav.driveTo(odo.getPosition(), SAMPLE_1, 0.6, 0.5)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                        while (linearSlideMotor.getCurrentPosition() > slidePosDown) {
                            linearSlideMotor.setPower(-1);
                        }
                        // pick up
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        flipServo.setPosition(flipPosDown);
                        sleep(100);
                        pivotServo.setPosition(pivotPosDown);
                        sleep(600);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        sleep(500);
                        // transfer
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        pivotServo.setPosition(pivotPosTransfer);
                        //sleep(500);
                        turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);
                        sleep(800);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        sleep(500);
                        pivotServo.setPosition(pivotPosFloat);
                        sleep(200);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() > slidePosDown) {
                            linearSlideMotor.setPower(-1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if(nav.driveTo(odo.getPosition(), BASKET_TARGET, 0.7, 0.5)){
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                        while (linearSlideMotor.getCurrentPosition() < slidePosUp) {
                            linearSlideMotor.setPower(1);
                        }
                        linearSlideMotor.setPower(0);
                        flipServo.setPosition(flipPosScore);
                        sleep(400);
                        flipServo.setPosition(flipPosDown);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < slidePosUp) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    //drive to the second target
                    if (nav.driveTo(odo.getPosition(), SAMPLE_2, 0.6, 0.5)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                        while (linearSlideMotor.getCurrentPosition() > slidePosDown) {
                            linearSlideMotor.setPower(-1);
                        }
                        // pick up
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        flipServo.setPosition(flipPosDown);
                        sleep(500);
                        pivotServo.setPosition(pivotPosDown);
                        sleep(600);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        sleep(500);
                        // transfer
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        pivotServo.setPosition(pivotPosTransfer);
                        //sleep(500);
                        turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);
                        sleep(800);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        sleep(500);
                        pivotServo.setPosition(pivotPosFloat);
                        sleep(200);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() > slidePosDown) {
                            linearSlideMotor.setPower(-1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if(nav.driveTo(odo.getPosition(), BASKET_TARGET, 0.7, 0.5)){
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;
                        while (linearSlideMotor.getCurrentPosition() < slidePosUp) {
                            linearSlideMotor.setPower(1);
                        }
                        linearSlideMotor.setPower(0);
                        flipServo.setPosition(flipPosScore);
                        sleep(400);
                        flipServo.setPosition(flipPosDown);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < slidePosUp) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    //drive to the second target
                    if (nav.driveTo(odo.getPosition(), SAMPLE_3, 0.6, 0.5)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;
                        while (linearSlideMotor.getCurrentPosition() > slidePosDown) {
                            linearSlideMotor.setPower(-1);
                        }
                        // pick up
                        gripperServo1.setPosition(0.55);
                        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
                        flipServo.setPosition(flipPosDown);
                        sleep(500);
                        pivotServo.setPosition(pivotPosDown);
                        sleep(600);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        sleep(500);
                        // transfer
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
                        pivotServo.setPosition(pivotPosTransfer);
                        //sleep(500);
                        turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);
                        sleep(800);
                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                        sleep(500);
                        pivotServo.setPosition(pivotPosFloat);
                        sleep(200);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() > slidePosDown) {
                            linearSlideMotor.setPower(-1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_7:
                    if(nav.driveTo(odo.getPosition(), BASKET_TARGET, 0.7, 0.5)){
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.AT_TARGET;
                        while (linearSlideMotor.getCurrentPosition() < slidePosUp) {
                            linearSlideMotor.setPower(1);
                        }
                        linearSlideMotor.setPower(0);
                        flipServo.setPosition(flipPosScore);
                        sleep(400);
                        flipServo.setPosition(flipPosDown);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < slidePosUp) {
                            linearSlideMotor.setPower(1);
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
                if (linearSlideMotor.getCurrentPosition() > 0) {
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
}
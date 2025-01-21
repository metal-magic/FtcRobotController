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

@Autonomous(name="Competition Auto Right", group="Pinpoint")
//@Disabled

public class CompetitionAutoRight extends LinearOpMode {

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
        DRIVE_TO_TARGET_16, DRIVE_TO_TARGET_17, DRIVE_TO_TARGET_15
    }

    static final Pose2D startingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // Starting position
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,-742,-290,AngleUnit.DEGREES,0); // Specimen Chamber 1
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, -750, -290, AngleUnit.DEGREES, 0); // Specimen Chamber 2
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,-400,582, AngleUnit.DEGREES,0); // April Tag scanning
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, -1340, 635, AngleUnit.DEGREES, 0); // April Tag Position
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, -1240, 830, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_6 = new Pose2D(DistanceUnit.MM, -260, 830, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_7 = new Pose2D(DistanceUnit.MM, -1340, 830, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_8 = new Pose2D(DistanceUnit.MM, -1250, 1090, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_9 = new Pose2D(DistanceUnit.MM, -350, 942, AngleUnit.DEGREES, 170);
    static final Pose2D TARGET_10 = new Pose2D(DistanceUnit.MM, -165, 942, AngleUnit.DEGREES, 170);
    static final Pose2D TARGET_11 = new Pose2D(DistanceUnit.MM, -60,    942 , AngleUnit.DEGREES, 170);
    static final Pose2D WAYPOINT_CHAMBER = new Pose2D(DistanceUnit.MM, -184, -74.614, AngleUnit.DEGREES, 0);
    static final Pose2D CHAMBER_NEW = new Pose2D(DistanceUnit.MM,-770,-74,AngleUnit.DEGREES,0);
    static final Pose2D CHAMBER_WAYPOINT2 = new Pose2D(DistanceUnit.MM, -670, -74.614, AngleUnit.DEGREES, 0);
    static final Pose2D CHAMBER_WAYPOINT3 = new Pose2D(DistanceUnit.MM, -400, -320, AngleUnit.DEGREES, 0);
    static final Pose2D CHAMBER_WAYPOINT4 = new Pose2D(DistanceUnit.MM, -740, -340, AngleUnit.DEGREES, 0);
    static final Pose2D CHAMBER_WAYPOINT5 = new Pose2D(DistanceUnit.MM, -750, -340, AngleUnit.DEGREES, 0);
    static final Pose2D NEW_CHAMBER = new Pose2D(DistanceUnit.MM, -784, -340, AngleUnit.DEGREES, 0);

    static final double slidePosDown = UtilityValues.SLIDE_POS_DOWN;
    static final double slidePosSpecDown = 2200; //UtilityValues.SLIDE_POS_SPEC_DOWN;
    static final double slidePosSpecUp = UtilityValues.SLIDE_POS_SPEC_UP;
    static final double slidePosUp = UtilityValues.SLIDE_POS_SAMP;
    static final double slidePosStable = UtilityValues.SLIDE_POS_STABLE;

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

    public boolean condition = false;

    public Pose2D currPose;


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
                    if (nav.driveTo(odo.getPosition(), TARGET_1, 0.4, 0)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                        while (linearSlideMotor.getCurrentPosition() < slidePosSpecUp) {
                            linearSlideMotor.setPower(1);
                        }
                        linearSlideMotor.setPower(0);
                        while (linearSlideMotor.getCurrentPosition() > slidePosSpecDown) {
                            linearSlideMotor.setPower(-1);
                        }
                        linearSlideMotor.setPower(0);
                        sleep(500);
                        clipServo.setPosition(clipPosOpen);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < slidePosSpecUp) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    //drive to the second target
                    if (nav.driveTo(odo.getPosition(), TARGET_2, 0.8, 0)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if(nav.driveTo(odo.getPosition(), TARGET_3, 0.8, 0)){
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
//                        if (!tagProcessor.getDetections().isEmpty()) {
//                            odo.setPosition(returnAprilTagPose(odo.getPosition()));
//                        }
                        odo.update();
                    } else {
                        if (linearSlideMotor.getCurrentPosition() > 50) {
                            linearSlideMotor.setPower(-1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if(nav.driveTo(odo.getPosition(),TARGET_4,0.725,0)){
                        telemetry.addLine("at position #4");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
//                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
//                        pivotServo.setPosition(pivotPosDown);
//                        sleep(1000);
//                        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
//                        sleep(200);
//                        pivotServo.setPosition(UtilityValues.PIVOT_POS_HOVER);
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if(nav.driveTo(odo.getPosition(),TARGET_5,0.725,0)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if(nav.driveTo(odo.getPosition(),TARGET_6,0.6,0)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    }
                    break;
                case DRIVE_TO_TARGET_7:
                    if(nav.driveTo(odo.getPosition(),TARGET_7,0.725,0)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;
                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    if(nav.driveTo(odo.getPosition(),TARGET_8,0.725,0)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    }
                    break;
                case DRIVE_TO_TARGET_9:
                    if(nav.driveTo(odo.getPosition(),TARGET_9,0.55,0)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_11;
                    }
                    break;
                case DRIVE_TO_TARGET_10:
                    if(nav.driveTo(odo.getPosition(),TARGET_10,0.7,0)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.AT_TARGET;
                        leftFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightBackDrive.setPower(0);
                    }
                    break;
                case DRIVE_TO_TARGET_11:
                    if(nav.driveTo(odo.getPosition(),TARGET_11,0.4,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_12;
                        leftFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        clipServo.setPosition(clipPosClose);
                        sleep(800);
                        while (linearSlideMotor.getCurrentPosition() < 500) {
                            linearSlideMotor.setPower(1);
                        }
                        linearSlideMotor.setPower(0.5);
                    }
                    break;
                case DRIVE_TO_TARGET_12:
                    if(nav.driveTo(odo.getPosition(), CHAMBER_WAYPOINT3,0.8,0)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_13;
                        if (linearSlideMotor.getCurrentPosition() < slidePosStable) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_13:
                    if (nav.driveTo(odo.getPosition(), CHAMBER_WAYPOINT4, 0.55, 0.1)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_14;
                    }
                    break;
                case DRIVE_TO_TARGET_14:
                    if (condition) {
                        currPose = NEW_CHAMBER;
                    } else {
                        currPose = CHAMBER_WAYPOINT5;
                    }
                    if(nav.driveTo(odo.getPosition(), currPose,0.6,0.5)){
                        telemetry.addLine("ThereCHECK!");
                        telemetry.addData("SlidePos", linearSlideMotor.getCurrentPosition());
                        if (condition) {
                            stateMachine = StateMachine.DRIVE_TO_TARGET_10;
                        } else {
                            stateMachine = StateMachine.DRIVE_TO_TARGET_15;
                        }
                        while (true) {
                            if (linearSlideMotor.getCurrentPosition() < slidePosSpecDown) {
                                linearSlideMotor.setPower(1);
                            } else if (linearSlideMotor.getCurrentPosition() < slidePosSpecUp) {
                                linearSlideMotor.setPower(0.5);
                            } else {
                                break;
                            }
                        }
                        linearSlideMotor.setPower(0);
                        sleep(250);
                        while (linearSlideMotor.getCurrentPosition() > slidePosSpecDown) {
                            linearSlideMotor.setPower(-1);
                        }
                        linearSlideMotor.setPower(0);
                        sleep(250);
                        clipServo.setPosition(clipPosOpen);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < slidePosSpecUp) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    break;
                    }
                case DRIVE_TO_TARGET_15:
                    if (nav.driveTo(odo.getPosition(), TARGET_10, 0.625, 0)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_11;
                        condition = true;
                    } else {
                        if (linearSlideMotor.getCurrentPosition() > 50) {
                            linearSlideMotor.setPower(-1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_16:
                    if (nav.driveTo(odo.getPosition(), TARGET_10, 0.7, 0)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;

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


}
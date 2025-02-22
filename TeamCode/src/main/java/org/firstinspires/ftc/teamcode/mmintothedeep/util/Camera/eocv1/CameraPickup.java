/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 *] of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.mmintothedeep.util.Camera.eocv1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.DriveToPoint;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.TeleOpTestToBasket;

import java.util.ArrayList;
import java.util.Date;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/*
  =========================================k
  This OpMode was created to test the linear slide motor
  =========================================
 */

@TeleOp(name= "Camera Test TeleOp WORLDS???")
public class CameraPickup extends LinearOpMode {

    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private ColorBlobLocatorProcessor colorLocator;
    private int myExposure;
    private int myGain;

    int newTarget;

    public Servo gripperServo1 = null;
    // public Servo gripperServo2 = null;
    //public Servo pivotServo = null;
    public Servo turnServo = null;
    public Servo clipServo = null;
    public Servo flipServo = null;
    public DcMotor linearSlideMotor = null;
    DcMotor leftFrontDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightBackDrive = null;

    DcMotor pivotMotor = null;

    public DcMotor hangSlideMotor1 = null;
    public DcMotor hangSlideMotor2 = null;

    public Date previousTime = new Date();

    public float armSpeedCounter = 0;
    // TouchSensor touchSensor = null;
    // OldSensorColor2 board = new OldSensorColor2();

    public static float setPositionCounter = 0;

    static final double MOTOR_TICK_COUNTS = UtilityValues.motorTicks; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the
    // wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER_INCHES = UtilityValues.wheelDiameter / 25.4; // in Inches
    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in
    // inches

    static final double DEGREES_MOTOR_MOVES_IN_1_REV = 45.0;

    static final double SPEED = UtilityValues.SPEED; // Motor Power setting

    boolean intake = false;
    long setTime = 0;

    boolean slideUp = false;
    boolean slideDown = false;

    static final double slidePosDown = UtilityValues.SLIDE_POS_DOWN;
    static final double slidePosSpecDown = UtilityValues.SLIDE_POS_SPEC_DOWN;
    static final double slidePosSpecUp = UtilityValues.SLIDE_POS_SPEC_UP;
    static final double slidePosUp = UtilityValues.SLIDE_POS_SAMP;
    static final double slidePosStable = UtilityValues.SLIDE_POS_STABLE;

    static final double pivotPosDown = UtilityValues.PIVOT_POS_DOWN;
    static final double pivotPosHover = UtilityValues.PIVOT_POS_HOVER;
    static final double pivotPosFloat = UtilityValues.PIVOT_POS_FLOAT;
    static final double pivotPosTransfer = UtilityValues.PIVOT_POS_TRANSFER;

    static final double pivotPosHang = UtilityValues.PIVOT_POS_HANG;

    static final double turnPosDown = UtilityValues.TURN_POS_DOWN;
    static final double turnPosTransfer = UtilityValues.TURN_POS_TRANSFER;

    static final double flipPosDown = UtilityValues.FLIP_POS_DOWN;
    static final double flipPosScore = UtilityValues.FLIP_POS_SCORE;

    static final double gripperPosClose = UtilityValues.GRIPPER_POS_CLOSE;
    static final double gripperPosOpen = UtilityValues.GRIPPER_POS_OPEN;

    static final double clipPosClose = UtilityValues.CLIP_POS_CLOSE;
    static final double clipPosOpen = UtilityValues.CLIP_POS_OPEN;

    boolean slideMidUp = false;
    boolean slideMidDown = false;
    boolean slideStable = false;

    boolean leftBumper_isPressed = false;
    boolean leftBumper_wasPressed = false;
    int currGripperPos = 0;

    boolean rightBumper_isPressed = false;
    boolean rightBumper_wasPressed = false;
    int currSpecPos = 0;

    boolean isTransferring = false;

    long startTime = 0;

    boolean isPressedEndOHYE = false;

    static int slideTarget = 0;

    boolean stopWheels = false;

    boolean atTarget = false;
    boolean isHangPressed = false;

    public static int ratioY = 5;
    public static int ratioX = 3;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_CHAMBER,
        ALIGN_TO_SAMPLE
    }

    static final Pose2D startingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 90); // Starting position
    static final Pose2D CHAMBER_TARGET = new Pose2D(DistanceUnit.MM, -1270, 430, AngleUnit.DEGREES, -90);
    static final Pose2D sampleStartingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // Starting position for samples
    static Pose2D sample_pos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);

    public void runOpMode() {



        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        // gripperServo2 = hardwareMap.servo.get("gripperServo2");
        //pivotServo = hardwareMap.servo.get("pivotServo");
        //pivotServo.setDirection(Servo.Direction.REVERSE);

        turnServo = hardwareMap.servo.get("turnServo");
        //gripperServo1.setDirection(Servo.Direction.REVERSE);

        clipServo = hardwareMap.servo.get("clipServo");

        flipServo = hardwareMap.servo.get("flipServo");

        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_COLOR, 0.3);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        hangSlideMotor = hardwareMap.dcMotor.get("hangSlideMotor1");
//        hangSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        hangSlideMotor2 = hardwareMap.dcMotor.get("hangSlideMotor2");
//        hangSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setDirection(UtilityValues.compLeftBackDirection);
        leftFrontDrive.setDirection(UtilityValues.compLeftFrontDirection);
        rightBackDrive.setDirection(UtilityValues.compRightBackDirection);
        rightFrontDrive.setDirection(UtilityValues.compRightFrontDirection);

        hangSlideMotor1 = hardwareMap.dcMotor.get("hangSlideMotor1");
        hangSlideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangSlideMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        hangSlideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangSlideMotor2 = hardwareMap.dcMotor.get("hangSlideMotor2");
        hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangSlideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangSlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        hangSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        hangSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        hangSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        hangSlideMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
//        hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
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

        //pivotServo.setPosition(UtilityValues.PIVOT_POS_FLOAT);
        //runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT, );
        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
        gripperServo1.setPosition(gripperPosOpen);

        double startTimeAtStart = System.currentTimeMillis();
        while (linearSlideMotor.getCurrentPosition() > 50) {
            if (System.currentTimeMillis() > startTimeAtStart + 2000) {
                break;
            }
            linearSlideMotor.setPower(-1);
        }

        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flipServo.setPosition(flipPosDown);

        boolean resetPivot = false;
        boolean CutPower = false;
        double motorSpeed;
        while (opModeIsActive()) {


            odo.update();
            switch (stateMachine) {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    odo.setPosition(startingPos);
                    stateMachine = StateMachine.AT_TARGET;
                    break;
                case DRIVE_TO_TARGET_CHAMBER:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), CHAMBER_TARGET, 0.65, 0.5)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.AT_TARGET;
                        leftFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        linearSlideMotor.setPower(0);
                    } else {
                        if (linearSlideMotor.getCurrentPosition() < UtilityValues.SLIDE_POS_SPEC_UP) {
                            linearSlideMotor.setPower(1);
                        } else {
                            linearSlideMotor.setPower(0);
                        }
                    }
                    atTarget = false;
                    break;
                case ALIGN_TO_SAMPLE:
                    if (nav.driveTo(odo.getPosition(), sample_pos, 0.65, 0.5)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.AT_TARGET;
                        leftFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        linearSlideMotor.setPower(0);

                    }
                    break;
                case AT_TARGET:
                    atTarget = true;
                    break;
            }

            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            if (!atTarget) {
                if (gamepad1.back) {
                    stateMachine = StateMachine.AT_TARGET;
                    atTarget = true;
                } else {
                    leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                    rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                    leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                    rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
                }
            } else {
                if (gamepad1.dpad_left) {
                    pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    pivotMotor.setPower(-0.5);
                    resetPivot = true;
                } else if (gamepad1.dpad_right) {
                    pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    pivotMotor.setPower(0.5);
                    resetPivot = true;
                } else {
                    if (resetPivot) {
                        pivotMotor.setPower(0);
                    }
                    pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad1.dpad_down) {
                    pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetPivot = false;
                }

                // flip the bucket
                if (gamepad2.x) {
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    flipServo.setPosition(flipPosScore);
                    sleep(400);
                    flipServo.setPosition(flipPosDown);
                }

                if (stopWheels) {
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                }

                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(0);

                double y = -gamepad2.left_stick_y - gamepad1.left_stick_y / 2; // REVERSED -gamepad1.left_stick_y.gamestick so
                // gamepad1 can also do movement for hanging
                // making sure it doesnt go over 1 or -1
                if (y < -1) {
                    y = -1;
                } else if (y > 1) {
                    y = 1;
                }
                double x = gamepad2.left_stick_x + gamepad1.left_stick_x / 2; // gamepad1 can also do movement for hanging
                // making sure it doesnt go over 1 or -1
                if (x > 1) {
                    x = 1;
                } else if (x < -1) {
                    x = -1;
                }
                double rx = gamepad2.right_stick_x + gamepad1.right_stick_x / 2; // gamepad1 can also do movement for hanging
                // making sure it doesnt go over 1 or -1
                if (rx > 1) {
                    rx = 1;
                } else if (rx < -1) {
                    rx = -1;
                }

                if (gamepad1.x) {
                    linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                // Denominator is the largest motor power (abs value) or 1
                // This makes sure that the ratio stays the same
                // but only when at least one is out of range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;
                motorSpeed = 1;

                leftFrontDrive.setPower(frontLeftPower * motorSpeed);
                leftBackDrive.setPower(backLeftPower * motorSpeed);
                rightFrontDrive.setPower(frontRightPower * motorSpeed);
                rightBackDrive.setPower(backRightPower * motorSpeed);

                if (gamepad2.right_trigger >= 0.3F) {
                    motorSpeed = 0.3;
                }

                if (gamepad1.y || gamepad2.back) {
                    //pivotServo.setPosition(pivotPosFloat);
                    runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT, 0.4);
                }

                // slide control

                if (linearSlideMotor.getCurrentPosition() > 3500) {
                    linearSlideMotor.setPower(0);
                    slideDown = false;
                    slideUp = false;
                    slideStable = false;
                    slideMidUp = false;
                    slideMidDown = false;
                    isTransferring = false;
                }

                if (gamepad1.a) {
                    linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                    linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linearSlideMotor.setPower(0.8);
                } else if (gamepad1.b) { // linearSlideMotor.getCurrentPosition() > 50 &&
                    linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                    linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linearSlideMotor.setPower(-0.8);
                } else if (slideUp) {
                    if (linearSlideMotor.getCurrentPosition() < slidePosUp) {
                        if (linearSlideMotor.getCurrentPosition() > 3500) {
                            linearSlideMotor.setPower(0.8);
                        } else {
                            linearSlideMotor.setPower(1);
                        }
                    }
                    if (linearSlideMotor.getCurrentPosition() > slidePosUp) {
                        slideUp = false;
                        linearSlideMotor.setPower(0);
                    }
                } else if (slideStable) {
                    if (linearSlideMotor.getCurrentPosition() < slidePosStable) {
                        linearSlideMotor.setPower(1);
                    }
                    if (linearSlideMotor.getCurrentPosition() > slidePosStable) {
                        slideStable = false;
                        linearSlideMotor.setPower(0);
                    }
                } else if (slideDown && !slideMidDown) {
                    if (linearSlideMotor.getCurrentPosition() < slidePosDown) {
                        slideDown = false;
                        linearSlideMotor.setPower(0);
                        flipServo.setPosition(flipPosDown);
                    } else {
                        linearSlideMotor.setPower(-1);
                    }
                } else if (slideMidUp) {
                    if (linearSlideMotor.getCurrentPosition() < slidePosSpecUp) {
                        linearSlideMotor.setPower(0.8);
                    } else {
                        linearSlideMotor.setPower(0);
                        slideMidDown = false;
                        slideDown = false;
                        slideUp = false;
                        slideMidUp = false;
                        isTransferring = false;
                    }
                } else if (slideMidDown) {
                    if (linearSlideMotor.getCurrentPosition() > slidePosSpecDown) {
                        linearSlideMotor.setPower(-0.6);
                    } else {
                        linearSlideMotor.setPower(0);
                        slideMidDown = false;
                        clipServo.setPosition(UtilityValues.CLIP_POS_OPEN);
                        sleep(200);
                        slideDown = true;
                    }
                } else {
                    linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    linearSlideMotor.setPower(0);
                    if (linearSlideMotor.getCurrentPosition() > 3500) {
                        linearSlideMotor.setPower(0);
                    }
                }


                // Transfer and slide up
                if (gamepad2.dpad_up) {
                    if (linearSlideMotor.getCurrentPosition() > 300) {
                        linearSlideMotor.setPower(-1);
                    }
                    isTransferring = true;
                    flipServo.setPosition(flipPosDown);
                    gripperServo1.setPosition(gripperPosClose);
                    //pivotServo.setPosition(pivotPosTransfer);
                    runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER, 0.35);
                    // nah dont sleep(500);
                    turnServo.setPosition(turnPosTransfer);
                    while (linearSlideMotor.getCurrentPosition() < 300) {
                        linearSlideMotor.setPower(0.7);
                        moveRobot();
                    }
                    linearSlideMotor.setPower(0);
                    startTime = System.currentTimeMillis();
//                sleep(800);
//                gripperServo1.setPosition(gripperPosOpen);
//                sleep(500);
//                pivotServo.setPosition(pivotPosFloat);
//                sleep(500);
//                slideUp = false;
//                slideDown = false;
//                slideMidUp = false;
//                slideMidDown = false;
//                slideStable = true;
                }

                if (isTransferring) {
                    if (System.currentTimeMillis() > startTime + 2000.0) {
                        slideUp = true;
                        slideDown = false;
                        slideMidUp = false;
                        slideMidDown = false;
                        slideStable = false;
                        isTransferring = false;
                    } else if (System.currentTimeMillis() > startTime + 1600.0) {
                        //pivotServo.setPosition(pivotPosFloat);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT, 0.35);
                    } else if (System.currentTimeMillis() > startTime + 1200.0) {
                        gripperServo1.setPosition(gripperPosOpen);
                    }
                }

                // align position
                if (gamepad2.dpad_right) {
                    turnServo.setPosition(turnPosDown);
                    flipServo.setPosition(flipPosDown);

                    runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_ALIGN, 0.3);
                    //pivotServo.setPosition(pivotPosHover);
                    //pivotServo.setPosition(pivotPosHover);
                    //gripperServo1.setPosition(gripperPosOpen);

                    slideDown = linearSlideMotor.getCurrentPosition() > 100;
                    slideUp = false;
                    slideMidUp = false;
                    slideMidDown = false;
                    slideStable = false;
                    isTransferring = false;

                }
                // picking up for specimen into human player
                if (gamepad2.dpad_left) {
                    turnServo.setPosition(turnPosDown);
                    flipServo.setPosition(flipPosDown);

                    //pivotServo.setPosition(UtilityValues.PIVOT_POS_OUT_OF_SUBMERSIBLE);
                    runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_SUB, 0.3);
                    gripperServo1.setPosition(gripperPosClose);
                }
                // pick up
                if (gamepad2.dpad_down) {
                    gripperServo1.setPosition(gripperPosOpen);
                    turnServo.setPosition(turnPosDown);
                    flipServo.setPosition(flipPosDown);
                    sleep(100);
                    runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_DOWN, 0.3);
                    //pivotServo.setPosition(pivotPosDown);
                    sleep(200);
//                gripperServo1.setPosition(gripperPosClose);
                    slideDown = linearSlideMotor.getCurrentPosition() > 100;
                    slideUp = false;
                    slideMidUp = false;
                    slideMidDown = false;
                    slideStable = false;
                    isTransferring = false;
                }



                //slide up and flip
                if (gamepad1.back) {
                    slideUp = true;
                    slideMidDown = false;
                    slideDown = false;
                    slideMidUp = false;
                    slideStable = false;
                    isTransferring = false;
                    sleep(1500);
                    flipServo.setPosition(flipPosScore);
                    sleep(1000);
                    flipServo.setPosition(flipPosDown);
                }

                if (Math.abs(gripperServo1.getPosition() - 0.05) <= 0.05) {
                    currGripperPos = 0;
                } else {
                    currGripperPos = 1;
                }
                if (Math.abs(clipServo.getPosition() - 0.05) <= 0.05) {
                    currSpecPos = 0;
                } else {
                    currSpecPos = 1;
                }





                // Slide all the way up
                if (gamepad2.a) {
                    slideUp = true;
                    slideDown = false;
                    slideMidUp = false;
                    slideMidDown = false;
                    slideStable = false;
                }

                // Slide all the way down
                if (gamepad2.b) {
                    slideDown = linearSlideMotor.getCurrentPosition() > 100;
                    slideUp = false;
                    slideMidUp = false;
                    slideMidDown = false;
                    slideStable = false;
                    isTransferring = false;
                }


                if (gamepad2.left_bumper) {
                    clipServo.setPosition(clipPosOpen);
                    gripperServo1.setPosition(gripperPosOpen);
                }

                if (gamepad2.right_bumper) {
                    clipServo.setPosition(clipPosClose);
                    gripperServo1.setPosition(gripperPosClose);
                }

//                if (gamepad2.right_trigger > 0.3) {
//                    slideMidUp = true;
//                    slideMidDown = false;
//                    slideDown = false;
//                    slideUp = false;
//                    isTransferring = false;
//                    slideStable = false;
//                    clipServo.setPosition(clipPosClose);
//                }

//                if (gamepad2.left_trigger > 0.3) {
//                    slideMidDown = true;
//                    slideDown = false;
//                    slideUp = false;
//                    slideMidUp = false;
//                    isTransferring = false;
//                }

                if (gamepad1.right_bumper) {
                    hangSlideMotor1.setDirection(DcMotor.Direction.FORWARD);
                    hangSlideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hangSlideMotor1.setPower(-0.7);
                } else if (gamepad1.left_bumper) {
                    hangSlideMotor1.setDirection(DcMotor.Direction.FORWARD);
                    hangSlideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hangSlideMotor1.setPower(0.7);
                } else {
                    if (!isHangPressed) {
                        hangSlideMotor1.setPower(0);
                    }
                }

                if (gamepad1.right_trigger >= 0.3F) {
                    hangSlideMotor2.setDirection(DcMotor.Direction.REVERSE);
                    hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hangSlideMotor2.setPower(-0.7*(1459.0/2040.0));
                } else if (gamepad1.left_trigger >= 0.3F) {
                    hangSlideMotor2.setDirection(DcMotor.Direction.REVERSE);
                    hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hangSlideMotor2.setPower(0.7*(1459.0/2040.0));
                } else {
                    if (!isHangPressed) {
                        hangSlideMotor2.setPower(0);
                    }
                }

                if (gamepad1.dpad_up) {
                    isHangPressed = true;
                } else {
                    if (isHangPressed) {
                        hangSlideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        hangSlideMotor1.setPower(-0.7 * (1459.0/2050.0));
                        hangSlideMotor2.setPower(-0.7);
                        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_HANG, 0.34);
                    }
                }

                if (gamepad1.right_stick_button) {
                    runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_HANG, 0.34);
                }


                telemetry.addData("Pivot", pivotMotor.getCurrentPosition());
                // telemetry.addLine("Pivot: " + String.valueOf(pivotServo.getPosition()));
                telemetry.addLine("Turn: " + String.valueOf(turnServo.getPosition()));
                telemetry.addLine("Gripper: " + String.valueOf(gripperServo1.getPosition()));
                telemetry.addLine("Clip: " + String.valueOf(clipServo.getPosition()));
                telemetry.addLine("Flip: " + String.valueOf(flipServo.getPosition()));
                telemetry.addLine("Slide: " + String.valueOf(linearSlideMotor.getCurrentPosition()));
//            telemetry.addLine("Hang1: " + String.valueOf(hangSlideMotor.getCurrentPosition()));
//            telemetry.addLine("Hang2: " + String.valueOf(hangSlideMotor2.getCurrentPosition()));
                telemetry.addData("slideUp", slideUp);
                telemetry.addData("odoX", odo.getPosX());
                telemetry.addData("odoY", odo.getPosY());



                if (gamepad2.right_trigger >= 0.3F) {
                    cameraTelemetry();
                }

                if (gamepad2.left_trigger >= 0.3F) {
                    atTarget = false;
                    odo.setPosition(sampleStartingPos);
                    atTarget = false;
                    alignToSample();
                    //odo.setPosition(sample_pos);
                    stateMachine = StateMachine.ALIGN_TO_SAMPLE;
                    atTarget = false;
                }

                if (gamepad1.left_bumper) {
                    runToPosition(pivotMotor, 130, 0.34);
                }

                telemetry.update();
                if (gamepad2.y) {
                    clipServo.setPosition(clipPosClose);
                    linearSlideMotor.setPower(0);
                    leftBackDrive.setPower(0);
                    leftFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    sleep(200);
                    odo.setPosition(startingPos);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_CHAMBER;
                    atTarget = false;
                }

                if (gamepad1.left_bumper) {
                    ratioY += 1;
                }

                if (gamepad1.left_trigger >= 0.3F) {
                    ratioY -= 1;
                }

                if (gamepad1.right_bumper) {
                    ratioX += 1;
                }

                if (gamepad1.right_trigger >= 0.3F) {
                    ratioX -= 1;
                }
            }

        }
    }

    public void moveRobot() {
        double y = -gamepad2.left_stick_y - gamepad1.left_stick_y / 2; // REVERSED -gamepad1.left_stick_y.gamestick so
        // gamepad1 can also do movement for hanging
        // making sure it doesnt go over 1 or -1
        if (y < -1) {
            y = -1;
        } else if (y > 1) {
            y = 1;
        }
        double x = gamepad2.left_stick_x + gamepad1.left_stick_x / 2; // gamepad1 can also do movement for hanging
        // making sure it doesnt go over 1 or -1
        if (x > 1) {
            x = 1;
        } else if (x < -1) {
            x = -1;
        }
        double rx = gamepad2.right_stick_x + gamepad1.right_stick_x / 2; // gamepad1 can also do movement for hanging
        // making sure it doesnt go over 1 or -1
        if (rx > 1) {
            rx = 1;
        } else if (rx < -1) {
            rx = -1;
        }

        if (gamepad1.x) {
            linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Denominator is the largest motor power (abs value) or 1
        // This makes sure that the ratio stays the same
        // but only when at least one is out of range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
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

    // Method to initialize colorBlobProcessor (Drawing boxes around samples)
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

    // Method to stream camera frames to the driver station
    private void cameraTelemetry() {
        telemetry.addData("preview on/off", "... Camera Stream\n");

        // Read the current list
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(500, 500000, blobs);  // filter out very small blobs.

        /*
         * The list of Blobs can be sorted using the same Blob attributes as listed above.
         * No more than one sort call should be made.  Sorting can use ascending or descending order.
         *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
         *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
         *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
         */

        telemetry.addLine(" Area Density Aspect  Center");
        sleep(200);
        // Display the size (area) and center location for each Blob.
        for(ColorBlobLocatorProcessor.Blob b : blobs)
        {
            RotatedRect boxFit = b.getBoxFit();
            telemetry.addLine(String.valueOf(b.getContourArea()));
            telemetry.addLine(String.valueOf(b.getDensity()));
            telemetry.addLine(String.valueOf(b.getAspectRatio()));
            telemetry.addLine(String.valueOf((int) boxFit.center.x));
            telemetry.addLine(String.valueOf((int) boxFit.center.y));
        }

        telemetry.update();
        sleep(20);
    }

    private void alignToSample() {

        // Read the current list
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(2000, 50000, blobs);  // filter out very small blobs.

        while (blobs.isEmpty()) {
            blobs = colorLocator.getBlobs();
        }

        double offsetX = 360.0;
        double offsetY = 240.0;

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

        ColorBlobLocatorProcessor.Blob bestBlob = blobs.get(index);
        RotatedRect boxFit = bestBlob.getBoxFit();

        sample_pos = new Pose2D(DistanceUnit.INCH, (offsetY - (int) boxFit.center.y) / offsetY * ratioY, -1 * ((int) boxFit.center.x - offsetX) / offsetX * ratioX, AngleUnit.DEGREES, 0);

        telemetry.addData("lowestScore", lowestScore);

        telemetry.addData("Pose X", sample_pos.getX(DistanceUnit.INCH));
        telemetry.addData("Pose Y", sample_pos.getY(DistanceUnit.INCH));
        telemetry.addData("Pose Heading", sample_pos.getHeading(AngleUnit.DEGREES));

        telemetry.update();
        sleep(20);
    }
}
package org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint;

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

import java.util.List;
import java.util.Locale;

@TeleOp(name = "TeleOp To Basket")
//@Disabled

/*
 * Try to go to the basket in teleop using odo
 */

public class TeleOpTestToBasket extends LinearOpMode {

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
    double motorSpeed;

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
        DRIVE_TO_TARGET_BASKET,
        DRIVE_TO_TARGET_SUB_WAYPOINT,
        DRIVE_TO_TARGET_SUBMERSIBLE
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
    static final Pose2D BASKET_TARGET = new Pose2D(DistanceUnit.MM, -450, 110, AngleUnit.DEGREES, 40);
    static final Pose2D SAMPLE_1 = new Pose2D(DistanceUnit.MM, -302, 450, AngleUnit.DEGREES, 90);
    static final Pose2D SAMPLE_2 = new Pose2D(DistanceUnit.MM, -522, 450, AngleUnit.DEGREES, 90);
    static final Pose2D SAMPLE_3 = new Pose2D(DistanceUnit.MM, -515, 570, AngleUnit.DEGREES, 120);
    static final Pose2D SUBMERSIBLE_TARGET = new Pose2D(DistanceUnit.MM, 300, 1569, AngleUnit.DEGREES, 0);
    static final Pose2D SUB_WAYPOINT_TARGET = new Pose2D(DistanceUnit.MM, -254, 1424, AngleUnit.DEGREES, 0);

    boolean atTarget = false;

    @Override
    public void runOpMode() {

//        initPortal();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

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

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();
            switch (stateMachine) {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    odo.setPosition(startingPos);
                    stateMachine = StateMachine.AT_TARGET;
                    break;
                case DRIVE_TO_TARGET_BASKET:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), BASKET_TARGET, 0.65, 0.5)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.AT_TARGET;

                        leftFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightBackDrive.setPower(0);
                    }
                    atTarget = false;
                    break;
                case DRIVE_TO_TARGET_SUB_WAYPOINT:
                    if (nav.driveTo(odo.getPosition(), SUB_WAYPOINT_TARGET, 1, 0.5)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_SUBMERSIBLE;

                        leftFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightBackDrive.setPower(0);
                    }
                    atTarget = false;
                    break;
                case DRIVE_TO_TARGET_SUBMERSIBLE:
                    if (nav.driveTo(odo.getPosition(), SUBMERSIBLE_TARGET, 0.3, 0.5)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.AT_TARGET;

                        leftFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightBackDrive.setPower(0);
                    }
                    atTarget = false;
                    break;
                case AT_TARGET:
                    atTarget = true;

                    break;
            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            if (!atTarget) {
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            } else {
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

                if (gamepad2.x) {
                    stateMachine = StateMachine.DRIVE_TO_TARGET_BASKET;
                    atTarget = false;
                }
                if (gamepad2.y) {
                    stateMachine = StateMachine.DRIVE_TO_TARGET_SUB_WAYPOINT;
                    atTarget = false;
                }
            }

            telemetry.addData("current state:", stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);


            telemetry.update();

        }
    }
}
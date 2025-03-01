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

@Autonomous(name="!!!STATES - AUTO RIGHT 44444444+0+PARK using claw")
//@Disabled

public class AutoRightV4 extends LinearOpMode {

    int newTarget;

    int step = 1;

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;

    public Servo specimenServo = null;

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

    static final Pose2D startingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 90); // Starting position
    static final Pose2D CHAMBER_SPEC_1 = new Pose2D(DistanceUnit.MM,-721,-310,AngleUnit.DEGREES, 0);
    static final Pose2D WAYPOINT_1 = new Pose2D(DistanceUnit.MM,-509,449,AngleUnit.DEGREES,0);
    static final Pose2D WAYPOINT_2 = new Pose2D(DistanceUnit.MM,-1343,670,AngleUnit.DEGREES,0);
    static final Pose2D READY_PUSH_1 = new Pose2D(DistanceUnit.MM,-1219,883,AngleUnit.DEGREES,0);
    static final Pose2D PUSH_1 = new Pose2D(DistanceUnit.MM,-325,895,AngleUnit.DEGREES,0);
    static final Pose2D WAYPOINT_3 = new Pose2D(DistanceUnit.MM,-1326,928,AngleUnit.DEGREES,0);
    static final Pose2D READY_TO_PUSH_2 = new Pose2D(DistanceUnit.MM,-1200,1100,AngleUnit.DEGREES,0);
    static final Pose2D PUSH_2_AND_PICK = new Pose2D(DistanceUnit.MM,-15,1000,AngleUnit.DEGREES,0);
    static final Pose2D CHAMBER_SPEC_2 = new Pose2D(DistanceUnit.MM,-715 ,-260,AngleUnit.DEGREES,0);
    static final Pose2D CHAMBER_SPEC_3 = new Pose2D(DistanceUnit.MM,-718 ,-220,AngleUnit.DEGREES,0);
    static final Pose2D CHAMBER_SPEC_4 = new Pose2D(DistanceUnit.MM,-718 ,-170,AngleUnit.DEGREES,0);
    static final Pose2D NEW_PICK_UP = new Pose2D(DistanceUnit.MM,-30,967,AngleUnit.DEGREES,0);
    static final Pose2D WAYPOINT_4 = new Pose2D(DistanceUnit.MM,-331,-18,AngleUnit.DEGREES,0);
    static final Pose2D NEW_PICK_UP_4 = new Pose2D(DistanceUnit.MM,-50,967,AngleUnit.DEGREES,0);

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

        specimenServo = hardwareMap.servo.get("specPivot");

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
        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT_AUTO, 0.4);

        nav.setXYCoefficients(0.008, 0.00001, 20, DistanceUnit.MM, 30);

        while (opModeIsActive()) {
            odo.update();

            switch (stateMachine) {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    odo.setPosition(startingPos);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    clipServo.setPosition(UtilityValues.CLIP_POS_CLOSE);
                    break;
                case DRIVE_TO_TARGET_1:
                    if (nav.driveTo(odo.getPosition(), CHAMBER_SPEC_1, 0.65, 0.3)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;

                        powerOff();

                        specimenScore();

                    } else {
                        runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 0.3);
                        specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_UP);
                    }
                    break;

                case DRIVE_TO_TARGET_2:
                    if (nav.driveTo(odo.getPosition(), WAYPOINT_1, 0.65, 0)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;

                        powerOff();

                    } else {

                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if (nav.driveTo(odo.getPosition(), WAYPOINT_2, 0.65, 0)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;

                        powerOff();

                    } else {

                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if (nav.driveTo(odo.getPosition(), READY_PUSH_1, 0.65, 0)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;

                        powerOff();

                    } else {

                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if (nav.driveTo(odo.getPosition(), PUSH_1, 0.65, 0)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;

                        powerOff();

                    } else {

                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if (nav.driveTo(odo.getPosition(), WAYPOINT_3, 0.6, 0)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;

                        powerOff();

                    } else {

                    }
                    break;
                case DRIVE_TO_TARGET_7:
                    if (nav.driveTo(odo.getPosition(), READY_TO_PUSH_2, 0.65, 0)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;

                        powerOff();

                    } else {

                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    if (nav.driveTo(odo.getPosition(), PUSH_2_AND_PICK, 0.65, 0.3)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;

                        powerOff();
                        sleep(300);
                        clipServo.setPosition(clipPosClose);
                        sleep(300);
                        specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_UP);

                        step = 1;

                    } else {

                    }
                    break;
                case DRIVE_TO_TARGET_9:
                    if (step == 1) {
                        if (nav.driveTo(odo.getPosition(), WAYPOINT_4, 0.6, 0)) {
                            telemetry.addLine("at position #1!");
                            powerOff();
                            step++;

                        }
                    } else {
                        if (nav.driveTo(odo.getPosition(), CHAMBER_SPEC_2, 0.65, 0.1)) {
                            telemetry.addLine("at position #1!");
                            stateMachine = StateMachine.DRIVE_TO_TARGET_10;

                            powerOff();
                            specimenScore();
                            powerOff();

                            step = 1;

                        }
                    }
                    break;
                case DRIVE_TO_TARGET_10:
                    if (nav.driveTo(odo.getPosition(), NEW_PICK_UP, 0.65, 0.3)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_11;

                        powerOff();

                        sleep(300);
                        clipServo.setPosition(clipPosClose);
                        sleep(300);
                        specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_UP);

                        step = 1;

                    } else {

                    }
                    break;
                case DRIVE_TO_TARGET_11:
                    if (step == 1) {
                        if (nav.driveTo(odo.getPosition(), WAYPOINT_4, 0.6, 0)) {
                            telemetry.addLine("at position #1!");
                            powerOff();
                            step++;

                        }
                    } else {
                        if (nav.driveTo(odo.getPosition(), CHAMBER_SPEC_3, 0.65, 0.3)) {
                            telemetry.addLine("at position #1!");
                            stateMachine = StateMachine.DRIVE_TO_TARGET_12;

                            specimenScore();
                            powerOff();

                            step = 1;

                        }
                    }
                    break;
                case DRIVE_TO_TARGET_12:
                    if (nav.driveTo(odo.getPosition(), NEW_PICK_UP_4, 0.65, 0.3)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_13;

                        powerOff();

                        sleep(300);
                        clipServo.setPosition(clipPosClose);
                        sleep(300);
                        specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_UP);

                        step = 1;

                    } else {

                    }
                    break;
                case DRIVE_TO_TARGET_13:
                    if (step == 1) {
                        if (nav.driveTo(odo.getPosition(), WAYPOINT_4, 0.6, 0)) {
                            telemetry.addLine("at position #1!");
                            powerOff();
                            step++;

                        }
                    } else {
                        if (nav.driveTo(odo.getPosition(), CHAMBER_SPEC_4, 0.65, 0.3)) {
                            telemetry.addLine("at position #1!");
                            stateMachine = StateMachine.AT_TARGET;
                            powerOff();
                            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_ALIGN_AUTO, 0.4);

                            specimenScore();

                            step = 1;

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

    public void powerOff() {
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void specimenScore2() {

        clipServo.setPosition(UtilityValues.CLIP_POS_LOOSEN);
        specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_SCORE);
        //sleepWithSlightly(1000);
        sleepWithSlightly(800, -0.5);
        clipServo.setPosition(UtilityValues.CLIP_POS_OPEN);
        specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_DOWN);

    }

    public void specimenScore() {

        clipServo.setPosition(UtilityValues.CLIP_POS_LOOSEN_TELEOP);
        specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_SCORE);
        //sleepWithSlightly(1000);
        sleepWithSlightly(400, -0.6);
        sleepWithSlightly(400, 0.3);
        clipServo.setPosition(UtilityValues.CLIP_POS_OPEN);
        specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_DOWN);

    }

    public void sleepWithSlightly(int miliseconds, double power) {
        double startTime = System.currentTimeMillis();
        double endTimer = startTime + miliseconds;
        while(System.currentTimeMillis() < endTimer) {
            moveRobotSlightly(power);
        }
    }

    public void moveRobotSlightly(double power) {
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
    }

}
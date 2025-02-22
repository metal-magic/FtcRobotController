package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.ForCompetition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.Utility;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

@TeleOp(name="!!!!! MAKE UP COMP TELEOP")
public class TeleOpForMakeUp extends LinearOpMode {

    public Servo gripperServo1 = null;
    public Servo turnServo = null;
    public Servo clipServo = null;
    public Servo flipServo = null;
    public Servo pivotServo = null;
    public DcMotor linearSlideMotor = null;

    DcMotor leftFrontDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightBackDrive = null;
    DcMotor pivotMotor = null;

    public int clawPosition = 0;
    public boolean wasPressedToggle = false;

    public int CLAWS_OPEN = 1;
    public int CLAWS_CLOSE = 0;

    public static long startTime = 0;

    public boolean isTransferring = false;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        afterStart();

        while (opModeIsActive()) {

            boolean transferButton = gamepad2.right_trigger > 0.3;
            boolean alignButton = gamepad2.left_trigger > 0.3;
            boolean downButton = gamepad2.right_bumper;
            boolean slideResetButton = gamepad1.x;
            boolean clawToggleButton = gamepad2.right_bumper;
            boolean specimenUpButton = gamepad2.dpad_up;
            boolean specimenDownButton = gamepad2.dpad_down;
            boolean specimenPickUpButton = gamepad2.dpad_right;

            moveRobot();
            slidePositions(transferButton, alignButton, downButton, slideResetButton);
            claws(clawToggleButton);
            specimenScore(specimenUpButton, specimenDownButton, specimenPickUpButton);
            isTransferring(isTransferring);

        }


    }

    public void claws(boolean toggle) {

        // toggle
        if (toggle && !wasPressedToggle) {
            clawPosition = (clawPosition + 1) % 2;
        }

        // servo open and close
        if (clawPosition == CLAWS_OPEN) {
            gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
            clipServo.setPosition(UtilityValues.CLIP_POS_OPEN);
        } else if (clawPosition == CLAWS_CLOSE){
            gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
            clipServo.setPosition(UtilityValues.CLIP_POS_CLOSE);
        }

        wasPressedToggle = toggle;
    }

    public void specimenScore(boolean specUp, boolean specDown, boolean specMiddle) {
        // pivot specimen
        if (specUp) {
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_UP);
        }

        if (specDown) {
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_SCORE);
            //sleepWithSlightly(1000);
            sleepWithSlightly(1000, 0.3);
            clawPosition = CLAWS_OPEN;
            clipServo.setPosition(UtilityValues.CLIP_POS_OPEN);
            sleepWithMoving(200);
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_DOWN);
        }

        if (specMiddle) {
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_DOWN);
            clawPosition = CLAWS_OPEN;
            clipServo.setPosition(UtilityValues.CLIP_POS_OPEN);
        }

    }

    public void sleepWithMoving(int miliseconds) {
        double startTime = System.currentTimeMillis();
        double endTimer = startTime + miliseconds;
        while(System.currentTimeMillis() < endTimer) {
            moveRobot();
        }
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

    public void afterStart() {
        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);

        double startTimeAtStart = System.currentTimeMillis();
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        flipServo.setPosition(UtilityValues.FLIP_POS_DOWN);
        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_SUB, 0.3);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initialize() {
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

        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        turnServo = hardwareMap.servo.get("turnServo");
        clipServo = hardwareMap.servo.get("clipServo");
        flipServo = hardwareMap.servo.get("flipServo");
        pivotServo = hardwareMap.servo.get("specPivot");

    }

    public void slidePositions(boolean slideUpControl, boolean alignControl, boolean downControl, boolean resetSlideControl) {

        if (slideUpControl) {
            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 0.5);
            isTransferring = true;
            flipServo.setPosition(UtilityValues.FLIP_POS_DOWN);
            clawPosition = CLAWS_CLOSE;
            gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER, 0.6);
            turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);

            linearSlideMotor.setPower(0);
            startTime = System.currentTimeMillis();
        }

        if (alignControl) {
            isTransferring = false;
            turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
            flipServo.setPosition(UtilityValues.FLIP_POS_DOWN);
            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_DOWN, 1);
            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_ALIGN, 0.4);
        }

        if (downControl) {
            isTransferring = false;
            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_DOWN, 1);
            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_DOWN, 0.4);
        }

        if (resetSlideControl) {
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

    }

    public void isTransferring(boolean isTransferring) {
        if (isTransferring) {
            if (System.currentTimeMillis() > startTime + 1000.0) {
                runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_SAMP, 1);
            } else if (System.currentTimeMillis() > startTime + 650.0) {
                runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT, 0.6);
            } else if (System.currentTimeMillis() > startTime + 400.0) {
                clawPosition = CLAWS_OPEN;
                gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
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
        motor.setTargetPosition(ticks);
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

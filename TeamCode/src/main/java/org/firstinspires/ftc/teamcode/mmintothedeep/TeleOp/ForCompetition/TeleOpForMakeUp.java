package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.ForCompetition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;


/**
 * Created for States Makeup Competition
 * This is a new TeleOp (Driver controlled) with modularized code
 * <p>
 * Contributors:
 * Aryan Mathur
 * Srinandasai Ari
 */

@TeleOp(name="!!!!!!!!! MAKE UP COMP TELEOP")
public class TeleOpForMakeUp extends LinearOpMode {

    /**
     * instantiating all the values
     */
    public Servo gripperServo1 = null;
    public Servo turnServo = null;
    public Servo clipServo = null;
    public Servo flipServo = null;
    public Servo specimenServo = null;
    public DcMotor linearSlideMotor = null;

    DcMotor leftFrontDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightBackDrive = null;
    DcMotor pivotMotor = null;
    DcMotor hangSlideMotor1 = null;
    DcMotor hangSlideMotor2 = null;

    /**
     * other variables to be used in code
     */
    // for claws
    public int clawPosition = 0;
    public boolean wasPressedClaw = false;
    public int CLAWS_OPEN = 1;
    public int CLAWS_CLOSE = 0;

    // for transferring
    public static long startTime = 0;
    public boolean isTransferring = false;
    public boolean isLowTransferring = false;

    // for mode switching between sample and specimen mode
    public int mode = 0;
    public boolean wasPressedMode = false;

    boolean isHangPressed;

    /**
     * Main section of code -- like 'main' method
     * @throws InterruptedException - just in case
     */
    @Override
    public void runOpMode() throws InterruptedException {

        initialize(); // initializing everything

        waitForStart(); // waiting until driver clicks play button

        afterStart(); // some slide and servo movements after driver clicks start

        isHangPressed = false;

        while (opModeIsActive()) {

            boolean modeSwitchButton = gamepad2.dpad_down || gamepad1.dpad_down;

            boolean SAMPLE_MODE = mode == 0;
            boolean SPECIMEN_MODE = mode == 1;

            boolean transferButton = gamepad2.right_trigger > 0.1 && SAMPLE_MODE;
            boolean alignButton = gamepad2.left_bumper && SAMPLE_MODE;
            boolean downButton = gamepad2.left_trigger > 0.1 && SAMPLE_MODE;
            boolean subButton = gamepad2.dpad_left && SAMPLE_MODE;
            boolean lowBasketButton = gamepad2.y && SAMPLE_MODE;

            boolean slideResetButton = gamepad1.x;
            boolean clawToggleButton = gamepad2.right_bumper;

            boolean specimenUpButton = gamepad2.left_trigger > 0.1 && SPECIMEN_MODE;
            boolean specimenDownButton = gamepad2.right_trigger > 0.3 && SPECIMEN_MODE;
            boolean specimenPickUpButton = gamepad2.left_bumper && SPECIMEN_MODE;
            boolean flipButton = gamepad2.x;
            boolean pivotFloat = gamepad2.back || gamepad1.back;

            boolean slideFullyUpButton = gamepad2.a;
            boolean slideFullyDownButton = gamepad2.b;
            boolean slideUpFailSafeButton = gamepad1.a;
            boolean slideDownFailSafeButton = gamepad1.b;

            moveRobot();
            slidePositions(transferButton, lowBasketButton, alignButton, subButton, downButton, slideResetButton, flipButton, pivotFloat);
            claws(clawToggleButton);
            specimenScore(specimenUpButton, specimenDownButton, specimenPickUpButton);
            isTransferring(isTransferring);
            isLowTransferring(isLowTransferring);

            hangSlide();

            toggleMode(modeSwitchButton);

            slideFailSafe(slideFullyUpButton, slideFullyDownButton, slideUpFailSafeButton, slideDownFailSafeButton);

            if (SAMPLE_MODE) {
                telemetry.addLine("SAMPLE MODE");
            } else {
                telemetry.addLine("SPECIMEN MODE");
            }
            telemetry.addData("Scoring Slide", linearSlideMotor.getCurrentPosition());

            telemetry.update();

        }


    }

    public void hangSlide() {
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
                specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_UP);
            }
        }

        if (gamepad1.right_stick_button) {
            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_HANG, 0.34);
            specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_UP);
        }
    }

    public void slideFailSafe(boolean slideFullyUpButton, boolean slideFullyDownButton, boolean slideUpFailSafeButton, boolean slideDownFailSafeButton) {

        if (slideFullyUpButton) {
            isTransferring = false;
            isLowTransferring = false;
            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_SAMP, 0.7);
        }

        if (slideFullyDownButton) {
            isTransferring = false;
            isLowTransferring = false;
            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
        }

        if (slideUpFailSafeButton) {
            isTransferring = false;
            isLowTransferring = false;
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(0.7);
        } else if (slideDownFailSafeButton) {
            isTransferring = false;
            isLowTransferring = false;
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(-0.7);
        } else {
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

    public void toggleMode(boolean toggle) {
        // toggle
        if (toggle && !wasPressedMode) {
            mode = (mode + 1) % 2;
        }

        wasPressedMode = toggle;
    }

    public void claws(boolean toggle) {

        // toggle
        if (toggle && !wasPressedClaw) {
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

        wasPressedClaw = toggle;
    }

    public void specimenScore(boolean specUp, boolean specDown, boolean specMiddle) {
        // pivot specimen
        if (specUp) {
            specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_UP_TELE);
        }

        if (specDown) {
            specimenScore();
            clipServo.setPosition(UtilityValues.CLIP_POS_OPEN);
            clawPosition = CLAWS_OPEN;
        }

        if (specMiddle) {
            specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_DOWN);
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

    public void afterStart() {
        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
        gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
        specimenServo.setPosition(UtilityValues.SPECIMEN_PIVOT_DOWN);

        double startTimeAtStart = System.currentTimeMillis();
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (linearSlideMotor.getCurrentPosition() > 50) {
            if (System.currentTimeMillis() > startTimeAtStart + 2000) {
                break;
            }
            linearSlideMotor.setPower(-1);
            pivotMotor.setPower(0.1);
        }

        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 0.5);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flipServo.setPosition(UtilityValues.FLIP_POS_DOWN);
        runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_ALIGN, 0.3);
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
        specimenServo = hardwareMap.servo.get("specPivot");

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

    }

    public void slidePositions(boolean slideUpControl, boolean slideLowerBasketControl, boolean alignControl, boolean subPosition, boolean downControl, boolean resetSlideControl, boolean flip, boolean pivotFloat) {

        if (flip) {
            isTransferring = false;
            isLowTransferring = false;
            flipServo.setPosition(UtilityValues.FLIP_POS_SCORE);
            sleepWithMoving(500);
            flipServo.setPosition(UtilityValues.FLIP_POS_DOWN);
            sleepWithMoving(200);
            alignControl = true; // align then
        }

        if (slideUpControl) {
            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
            isTransferring = true;
            isLowTransferring = false;
            flipServo.setPosition(UtilityValues.FLIP_POS_DOWN);
            clawPosition = CLAWS_CLOSE;
            gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER, 0.5);
            turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);

            linearSlideMotor.setPower(0);
            startTime = System.currentTimeMillis();
        }

        if (slideLowerBasketControl) {
            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
            isLowTransferring = true;
            isTransferring = false;
            flipServo.setPosition(UtilityValues.FLIP_POS_DOWN);
            clawPosition = CLAWS_CLOSE;
            gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER, 0.5);
            turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);

            linearSlideMotor.setPower(0);
            startTime = System.currentTimeMillis();
        }

        if (slideUpControl) {
            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
            isLowTransferring = false;
            isTransferring = true;
            flipServo.setPosition(UtilityValues.FLIP_POS_DOWN);
            clawPosition = CLAWS_CLOSE;
            gripperServo1.setPosition(UtilityValues.GRIPPER_POS_CLOSE);
            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_TRANSFER, 0.5);
            turnServo.setPosition(UtilityValues.TURN_POS_TRANSFER);

            linearSlideMotor.setPower(0);
            startTime = System.currentTimeMillis();
        }

        if (alignControl) {
            isTransferring = false;
            isLowTransferring = false;
            turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
            flipServo.setPosition(UtilityValues.FLIP_POS_DOWN);
            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_ALIGN, 0.45);
        }

        if (subPosition) {
            isTransferring = false;
            isLowTransferring = false;
            turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
            flipServo.setPosition(UtilityValues.FLIP_POS_DOWN);
            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_SUB, 0.45);
        }

        if (downControl) {
            isTransferring = false;
            isLowTransferring = false;
            turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
            flipServo.setPosition(UtilityValues.FLIP_POS_DOWN);
            runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_DOWN, 0.45);
        }

        if (resetSlideControl) {
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (pivotFloat) {
            runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_SPEC, 0.4);
        }

    }

    public void isTransferring(boolean isTransferring) {
        if (isTransferring) {
            if (System.currentTimeMillis() > startTime + 1200.0) {
                runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_SAMP, 1);
                flipServo.setPosition(UtilityValues.FLIP_POS_MID);
            } else if (System.currentTimeMillis() > startTime + 850.0) {
                runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT, 0.6);
            } else if (System.currentTimeMillis() > startTime + 600.0) {
                clawPosition = CLAWS_OPEN;
                gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
            }
        }
    }

    public void isLowTransferring(boolean isLowTransferring) {
        if (isTransferring) {
            if (System.currentTimeMillis() > startTime + 1200.0) {
                runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_SAMP_LOWER_BASKET, 1);
                flipServo.setPosition(UtilityValues.FLIP_POS_MID);
            } else if (System.currentTimeMillis() > startTime + 850.0) {
                runToPosition(pivotMotor, UtilityValues.PIVOT_MOTOR_FLOAT, 0.6);
            } else if (System.currentTimeMillis() > startTime + 600.0) {
                clawPosition = CLAWS_OPEN;
                gripperServo1.setPosition(UtilityValues.GRIPPER_POS_OPEN);
                runToPosition(linearSlideMotor, (int) UtilityValues.SLIDE_POS_TRANSFER, 1);
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

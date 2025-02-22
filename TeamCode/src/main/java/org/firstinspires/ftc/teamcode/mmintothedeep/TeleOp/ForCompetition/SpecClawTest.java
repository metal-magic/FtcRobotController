package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.ForCompetition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

@TeleOp(name = "!! Test The Specimen Stuff")
public class SpecClawTest extends LinearOpMode {

    public Servo pivotServo = null;
    public Servo clipServo = null;

    DcMotor leftFrontDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightBackDrive = null;

    public void runOpMode() {

        pivotServo = hardwareMap.servo.get("specPivot");
        clipServo = hardwareMap.servo.get("clipServo");

        initDriveMotors();

        telemetry.addLine("initialized");

        waitForStart();

        pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_DOWN);

        while (opModeIsActive()) {
            runLoop();
            telemetry.addData("Spec Arm Position", pivotServo.getPosition());
//            telemetry.addData("Spec Arm Position", pivotServo.getPosition());
        }
    }

    public void runLoop() {
        moveRobot();

        // clip servo open and close
        if (gamepad2.right_bumper) {
            clipServo.setPosition(UtilityValues.CLIP_POS_CLOSE);
        }
        if (gamepad2.left_bumper) {
            clipServo.setPosition(UtilityValues.CLIP_POS_OPEN);
        }

        if (gamepad2.y) {
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_DOWN);
        }

        // pivot specimen
        if (gamepad2.dpad_up) {
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_UP);
        }
        if (gamepad2.dpad_down) {
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_SCORE);
            //sleepWithSlightly(1000);
            sleepWithSlightly(1000, 0.3);
            clipServo.setPosition(UtilityValues.CLIP_POS_OPEN);
            sleepWithMoving(200);
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_DOWN);
        }

        telemetry.addData("Spec Arm Position", pivotServo.getPosition());
    }

    public void initDriveMotors() {
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

    public void moveRobotSlightly(double power) {
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
    }

}

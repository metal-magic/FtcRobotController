package org.firstinspires.ftc.teamcode.mmintothedeep.util.DriveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import java.lang.Math;

import org.firstinspires.ftc.teamcode.mmcenterstage.autonomous.AutonomousUtility;
import org.firstinspires.ftc.teamcode.mmintothedeep.util.UtilityValues;

import java.util.Date;

public class DriveTrainDriver {
    /* Declare all motors as null */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    static final double MOTOR_TICK_COUNTS = UtilityValues.motorTicks; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER_INCHES = UtilityValues.wheelDiameter / 25.4; // in Inches
    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES;
    // pi * the diameter of the wheels in inches

    static final double DEGREES_MOTOR_MOVES_IN_1_REV = 45.0;

    static final double SPEED = 0.5; // Motor Power setting

    HardwareMap hardwareMap = null;

    /*
    ==============DriveTrainDriver Constructor==============
    fLeft, bLeft, fRight, bRight are for specific motors
    ========================================================
     */
    public DriveTrainDriver(HardwareMap hardwareMap, boolean fLeft, boolean bLeft, boolean fRight, boolean bRight) {
        this.hardwareMap = hardwareMap;
        if (fLeft) {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
            leftFrontDrive.setDirection(UtilityValues.leftFrontDirection);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftFrontDrive.setPower(0);
        }
        if (bLeft) {
            leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
            leftBackDrive.setDirection(UtilityValues.leftBackDirection);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftBackDrive.setPower(0);
        }
        if (fRight) {
            rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
            rightFrontDrive.setDirection(UtilityValues.rightFrontDirection);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFrontDrive.setPower(0);
        }
        if (bRight) {
            rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");
            rightBackDrive.setDirection(UtilityValues.rightBackDirection);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightBackDrive.setPower(0);
        }
    }

    public void moveStraightLine(double movementInInches, double robotSpeed) {
        double moveInRevs = movementInInches / CIRCUMFERENCE_INCHES;
        drive(robotSpeed, moveInRevs, moveInRevs, moveInRevs, moveInRevs);
    }

    public void strafe(double strafeInches, double robotSpeed) {
        // We assume that strafing right means positive
        double strafeRevs = Math.abs(strafeInches / CIRCUMFERENCE_INCHES);
        if (strafeInches >= 0) {
            drive(robotSpeed,
                    1 * strafeRevs,
                    -1 * strafeRevs,
                    -1 * strafeRevs,
                    1 * strafeRevs);
        } else {
            drive(robotSpeed,
                    -1 * strafeRevs,
                    1 * strafeRevs,
                    1 * strafeRevs,
                    -1 * strafeRevs);
        }
    }
    public void rotate(double degrees, double robotSpeed) {
        // Assume positive degrees means moving towards the right
        double movementOfWheelsInRevs = Math.abs(degrees / DEGREES_MOTOR_MOVES_IN_1_REV);

        if (degrees >= 0) {
            drive(robotSpeed,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs);
        } else {
            // Moving negative means rotating left
            drive(robotSpeed,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs);
        }
    }

    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs, double rightBackRevs) {

        int LFdrivetarget = (int) (leftFrontRevs * MOTOR_TICK_COUNTS) + leftFrontDrive.getCurrentPosition();
        int LBdrivetarget = (int) (leftBackRevs * MOTOR_TICK_COUNTS) + leftBackDrive.getCurrentPosition();
        int RFdrivetarget = (int) (rightFrontRevs * MOTOR_TICK_COUNTS) + rightFrontDrive.getCurrentPosition();
        int RBdrivetarget = (int) (rightBackRevs * MOTOR_TICK_COUNTS) +  rightBackDrive.getCurrentPosition();

        leftFrontDrive.setTargetPosition(LFdrivetarget);
        leftBackDrive.setTargetPosition(LBdrivetarget);
        rightFrontDrive.setTargetPosition(RFdrivetarget);
        rightBackDrive.setTargetPosition(RBdrivetarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void strafeDiagonalLeft(double strafeLeftInches, double robotSpeed) {
        double strafeLeftRevs = Math.abs(strafeLeftInches / CIRCUMFERENCE_INCHES);

        if (strafeLeftInches >= 0) {
            drive(robotSpeed,
                    0,
                    1 * strafeLeftRevs,
                    1 * strafeLeftRevs,
                    0);
        } else {
            drive(robotSpeed,
                    0,
                    -1 * strafeLeftRevs,
                    -1 * strafeLeftRevs,
                    0);
        }
    }

    public void strafeDiagonalRight(double strafeLeftInches, double robotSpeed) {
        double strafeLeftRevs = Math.abs(strafeLeftInches / CIRCUMFERENCE_INCHES);

        if (strafeLeftInches >= 0) {
            drive(robotSpeed,
                    1 * strafeLeftRevs,
                    0,
                    0,
                    1 * strafeLeftRevs);
        } else {
            drive(robotSpeed,
                    -1 * strafeLeftRevs,
                    0,
                    0,
                    -1 * strafeLeftRevs);
        }
    }

    public void strafeAnyAngle(double strafeInches, double strafeAngleDegrees, double robotSpeed) {
        if (strafeAngleDegrees > 0 && strafeAngleDegrees < 360) {
            double strafeAngleRadians = Math.PI/180 * strafeAngleDegrees;
            double strafeAnyRevs = Math.abs(strafeInches / CIRCUMFERENCE_INCHES);

            double strafeLeftFrontPower = Math.sin(strafeAngleRadians + Math.PI/4) * strafeAnyRevs;
            double strafeLeftBackPower = Math.sin(strafeAngleRadians - Math.PI/4) * strafeAnyRevs;
            double strafeRightFrontPower = Math.sin(strafeAngleRadians - Math.PI/4) * strafeAnyRevs;
            double strafeRightBackPower = Math.sin(strafeAngleRadians + Math.PI/4) * strafeAnyRevs;


            drive(robotSpeed, strafeLeftFrontPower, strafeLeftBackPower, strafeRightFrontPower, strafeRightBackPower);
        }

    }

    public void terminate() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

}

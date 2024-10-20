package org.firstinspires.ftc.teamcode.mmintothedeep.util.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.Utility;

import org.firstinspires.ftc.teamcode.mmintothedeep.util.UtilityValues;
import org.slf4j.helpers.Util;

public class DriveTrainUtil {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    static final double MOTOR_TICK_COUNTS = UtilityValues.motorTicks; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    public static final double WHEEL_DIAMETER_INCHES = UtilityValues.WHEEL_DIAMETER_INCHES; // in Inches
    public static final double CIRCUMFERENCE_INCHES = UtilityValues.CIRCUMFERENCE_INCHES;
    // pi * the diameter of the wheels in inches

    public static final double DEGREES_MOTOR_MOVES_IN_1_REV = UtilityValues.DEGREES_MOTOR_MOVES_IN_1_REV;

    public static final double SPEED = UtilityValues.SPEED; // Motor Power setting

    HardwareMap hardwareMap = null;

    /**
     * default constructor
     *
     */
    public DriveTrainUtil (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        //this.hardwareMap = hardwareMap;

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "motorFrontLeft");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "motorBackRight");
        // Set all the right motor directions
        leftFrontDrive.setDirection(UtilityValues.leftFrontDirection);
        leftBackDrive.setDirection(UtilityValues.leftBackDirection);
        rightFrontDrive.setDirection(UtilityValues.rightFrontDirection);
        rightBackDrive.setDirection(UtilityValues.rightBackDirection);

        // Reset encoders positions
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
    }

    /**
     * Drives the robot forward based on a value in inches
     * @param movementInInches - how far you want robot to move
     */
    public void moveStraightLine(double movementInInches) {
        double moveInRevs = movementInInches / CIRCUMFERENCE_INCHES;
        drive(SPEED, moveInRevs, moveInRevs, moveInRevs, moveInRevs);
    }

    /**
     * Strafes the robot sideways based on a value in inches
     * @param strafeInches - how far you want robot to move (+ is right, - is left)
     */
    public void strafe(double strafeInches) {
        // We assume that strafing right means positive
        double strafeRevs = Math.abs(strafeInches / CIRCUMFERENCE_INCHES);
        if (strafeInches >= 0) {
            drive(SPEED,
                    1 * strafeRevs,
                    -1 * strafeRevs,
                    -1 * strafeRevs,
                    1 * strafeRevs);
        } else {
            drive(SPEED,
                    -1 * strafeRevs,
                    1 * strafeRevs,
                    1 * strafeRevs,
                    -1 * strafeRevs);
        }
    }

    /**
     * Rotates the robot n degrees
     * @param degrees - how many degrees you want robot to rotate
     */
    public void rotate(double degrees) {
        // Assume positive degrees means moving towards the right
        double movementOfWheelsInRevs = Math.abs(degrees / DEGREES_MOTOR_MOVES_IN_1_REV);

        if (degrees >= 0) {
            drive(SPEED,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs);
        } else {
            // Moving negative means rotating left
            drive(SPEED,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs);
        }
    }

    /**
     * Common drive class for individual motors
     * @param speed - speed of robot
     * @param leftFrontRevs - left front revs
     * @param leftBackRevs - left back revs
     * @param rightFrontRevs - right front revs
     * @param rightBackRevs - right back revs
     */
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

}

package org.firstinspires.ftc.teamcode.mmcenterstage.other;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class NanduDriveTrain {
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    static final double WHEEL_DIAMETER_INCHES = 96/25.4;

    static final double MOTOR_TICK_COUNTS = 500;

    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES;

    static final double SPEED = 0.5;

    HardwareMap hardwareMap = null;

    public void initializeDriveTrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        frontLeftDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motorbackLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "motorbackRight");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        // Reset encoders positions
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

    }

    public void moveStraightLine (double movementInInches) {
        double moveInRevs = movementInInches / CIRCUMFERENCE_INCHES;
        drive(SPEED, moveInRevs, moveInRevs, moveInRevs, moveInRevs);
    }

    private void strafe(double strafeInches) {
        // We assume that strafing right means positive
        double strafeRevs = Math.abs(strafeInches / CIRCUMFERENCE_INCHES);
        if (strafeInches > 0) {
            drive(SPEED,
                    1 * strafeRevs,
                    -1 * strafeRevs,
                    -1 * strafeRevs,
                    1 * strafeRevs);
        } else if (strafeInches < 0) {
            drive(SPEED,
                    -1 * strafeRevs,
                    1 * strafeRevs,
                    1 * strafeRevs,
                    -1 * strafeRevs);
        }
    }

    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs, double rightBackRevs) {

        int lfDriveTarget = (int) (leftFrontRevs * MOTOR_TICK_COUNTS) + frontLeftDrive.getCurrentPosition();
        int lbDriveTarget = (int) (leftBackRevs * MOTOR_TICK_COUNTS) + backLeftDrive.getCurrentPosition();
        int rfDriveTarget = (int) (rightFrontRevs * MOTOR_TICK_COUNTS) + frontRightDrive.getCurrentPosition();
        int rbDriveTarget = (int) (rightBackRevs * MOTOR_TICK_COUNTS) +  backRightDrive.getCurrentPosition();

        frontLeftDrive.setTargetPosition(lfDriveTarget);
        backLeftDrive.setTargetPosition(lbDriveTarget);
        frontRightDrive.setTargetPosition(rfDriveTarget);
        backRightDrive.setTargetPosition(rbDriveTarget);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        frontLeftDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backRightDrive.setPower(speed);

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

    }

    
}

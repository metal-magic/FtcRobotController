package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java. lang. Math;
@Autonomous(name="Robot: Auto Drive By Encoder THREE", group="Robot")
public class TestAutoWithEncoders extends LinearOpMode {

    public static final double MOTOR_SPEED = 0.5;
    /* Declare all motors as null */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    static final double MOTOR_TICK_COUNTS = 537.7; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER = 96 / 25.4; // in Inches
    static final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // pi * the diameter of the wheels in inches

    @Override
    public void runOpMode() {
        /* Assign all the motors */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Set all the right motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // Reset encoders positions
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        drive(MOTOR_SPEED, 88, 88, 88, 88);
        drive(0.5, 13.7444678595, 13.7444678595, -13.7444678595, -13.7444678595);
        drive(0.5, 88, 88, 88, 88);
        drive(0.5, 88, -88, -88, 88);
        drive(0.5, 27.488935719, 27.488935719, -27.488935719, -27.488935719);
        drive(0.5, 88, 88, 88, 88);
        drive(0.5, 20.6167017893, 20.6167017893, -20.6167017893, -20.6167017893);
        drive(0.5, 62.2253967444, 62.2253967444, 62.2253967444, 62.2253967444);
        drive(0.5, -54.977871438, -54.977871438, 54.977871438, 54.977871438);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);




    }

    private void rotate(double degrees){
        // Assume positive degrees means moving towards the right
        double movement_of_wheel_in_inches = Math.abs(degrees / 360 * CIRCUMFERENCE);

        if (degrees > 0){
            drive(0.5, movement_of_wheel_in_inches, movement_of_wheel_in_inches, -1 * movement_of_wheel_in_inches, -1 * movement_of_wheel_in_inches);
        }else{

        }
    }
    public void drive(double speed, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches) {

        double LFrotation = leftFrontInches / CIRCUMFERENCE;
        double LBrotation = leftBackInches / CIRCUMFERENCE;
        double RFrotation = rightFrontInches / CIRCUMFERENCE;
        double RBrotation = rightBackInches / CIRCUMFERENCE;

        int LFdrivetarget = (int)(LFrotation * MOTOR_TICK_COUNTS) + leftFrontDrive.getCurrentPosition();
        int LBdrivetarget = (int)(LBrotation * MOTOR_TICK_COUNTS) + leftBackDrive.getCurrentPosition();
        int RFdrivetarget = (int)(RFrotation * MOTOR_TICK_COUNTS) + rightFrontDrive.getCurrentPosition();
        int RBdrivetarget = (int)(RBrotation * MOTOR_TICK_COUNTS) + rightBackDrive.getCurrentPosition();

        leftFrontDrive.setTargetPosition(LFdrivetarget);
        leftBackDrive.setTargetPosition(LBdrivetarget);
        rightFrontDrive.setTargetPosition(RFdrivetarget);
        rightBackDrive.setTargetPosition(RBdrivetarget);

        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {
            telemetry.addData("All 4 wheels are running", '1');
            telemetry.update();
        }
        sleep(250);
    }
}

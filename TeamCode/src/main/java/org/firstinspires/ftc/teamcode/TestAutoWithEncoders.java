package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Robot: Auto Drive By Encoder THREE", group="Robot")
public class TestAutoWithEncoders extends LinearOpMode {

    /* Declare all motors as null */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    static final double MOTOR_TICK_COUNTS = 537.7; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER_INCHES = 96 / 25.4; // in Inches
    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches

    static final double SPEED = 0.5; // Motor Power setting

    @Override
    public void runOpMode() {
        /* Assign all the motors */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Set all the right motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // Reset encoders positions
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // ABOVE THIS, THE ENCODERS AND MOTOR ARE NOW RESET

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

      /*
        ============================
        THIS IS THE ACTUAL DRIVING
        ============================
       */
        moveStraightLine(12);
        strafe(12);
        rotate(90);

//        moveStraightLine (88);
//        strafe(88);
//        rotate(180);
//        moveStraightLine(88);
//        rotate(45);
//        moveStraightLine(62.2253967444);
//        rotate(-360);

//        drive(SPEED, 88, 88, 88, 88);
//        drive(SPEED, 13.7444678595, 13.7444678595, -13.7444678595, -13.7444678595);
//        drive(SPEED, 88, 88, 88, 88);
//        drive(SPEED, 88, -88, -88, 88);
//        drive(SPEED, 27.488935719, 27.488935719, -27.488935719, -27.488935719);
//        drive(SPEED, 88, 88, 88, 88);
//        drive(SPEED, 20.6167017893, 20.6167017893, -20.6167017893, -20.6167017893);
//        drive(SPEED, 62.2253967444, 62.2253967444, 62.2253967444, 62.2253967444);
//        drive(SPEED, -54.977871438, -54.977871438, 54.977871438, 54.977871438);

//        leftFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        rightBackDrive.setPower(0);

    }

    /*
    =====================================================
    PROGRAMMING FUNCTIONS FOR THE SEPARATE MOVEMENT TYPES
    =====================================================
     */
    private void strafe(double strafeInches) {
        // We assume that strafing right means positive
        if (strafeInches >= 0) {
            telemetry.addData("Strafing towards right by ", "%.3f inches", strafeInches);
            telemetry.update();
            drive(SPEED,
                    1 * strafeInches,
                    -1 * strafeInches,
                    -1 * strafeInches,
                    1 * strafeInches);
        } else {
            telemetry.addData("Strafing towards Left by ", "%.3f inches", Math.abs(strafeInches));
            telemetry.update();
            drive(SPEED,
                    -1 * strafeInches,
                    1 * strafeInches,
                    1 * strafeInches,
                    -1 * strafeInches);
        }
    }

    private void moveStraightLine(double movementInInches) {
        telemetry.addData("Moving ", "%.3f inches", movementInInches);
        telemetry.update();
        drive(SPEED, movementInInches, movementInInches, movementInInches, movementInInches);
    }


    /**
     * Function to Rotate the 4-Wheel Robot by certain amount of degrees.
     *
     * @param degrees POSITIVE degrees means rotating **RIGHT**
     */
    private void rotate(double degrees) {
        // Assume positive degrees means moving towards the right
        double movement_of_wheel_in_inches = Math.abs( 360 / degrees) * CIRCUMFERENCE_INCHES;

        if (degrees >= 0) {
            telemetry.addData("Rotating right by ", "%.3f inches", degrees);
            telemetry.update();
            drive(SPEED,
                    1.0 * movement_of_wheel_in_inches,
                    1.0 * movement_of_wheel_in_inches,
                    -1 * movement_of_wheel_in_inches,
                    -1 * movement_of_wheel_in_inches);
        } else {
            // Moving negative means rotating left
            telemetry.addData("Rotating left by ", "%.3finches", Math.abs(degrees));
            telemetry.update();
            drive(SPEED,
                    -1 * movement_of_wheel_in_inches,
                    -1 * movement_of_wheel_in_inches,
                    1.0 * movement_of_wheel_in_inches,
                    1.0 * movement_of_wheel_in_inches);
        }
    }

    public void drive(double speed, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches) {
        double LFrotation = leftFrontInches / CIRCUMFERENCE_INCHES;
        double LBrotation = leftBackInches / CIRCUMFERENCE_INCHES;
        double RFrotation = rightFrontInches / CIRCUMFERENCE_INCHES;
        double RBrotation = rightBackInches / CIRCUMFERENCE_INCHES;

        int LFdrivetarget = (int) (LFrotation * MOTOR_TICK_COUNTS) + leftFrontDrive.getCurrentPosition();
        int LBdrivetarget = (int) (LBrotation * MOTOR_TICK_COUNTS) + leftBackDrive.getCurrentPosition();
        int RFdrivetarget = (int) (RFrotation * MOTOR_TICK_COUNTS) + rightFrontDrive.getCurrentPosition();
        int RBdrivetarget = (int) (RBrotation * MOTOR_TICK_COUNTS) +  rightBackDrive.getCurrentPosition();

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

        while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
               telemetry.addLine("Current Position of the Motors")
                .addData("Left Front  ", "%d", leftFrontDrive.getCurrentPosition())
                .addData("Left Back ", "%d", leftBackDrive.getCurrentPosition())
                .addData("Right Front ", "%d", rightFrontDrive.getCurrentPosition())
                .addData("Right Back ", "%df", rightBackDrive.getCurrentPosition());

            telemetry.addLine("Target Positions of the Motors")
                    .addData("Left Front  ", "%d", LFdrivetarget)
                    .addData("Left Back ", "%d", LBdrivetarget)
                    .addData("Right Front ", "%d", RFdrivetarget)
                    .addData("Right Back ", "%df", RBdrivetarget);

            telemetry.update();
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        sleep(250);
    }
}

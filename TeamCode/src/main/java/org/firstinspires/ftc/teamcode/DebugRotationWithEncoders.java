package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The intention of this OpMode is to debug and find out
 * how many degrees the robot turns with 1 Rev of each of the 4 wheels
 * since its a 4-wheel drive train and the rotation axis is
 * at the intersection of the diagonals. We learnt through this experimentation
 * that 1 Rev move 45 degrees, 2 revs rotates 90 degrees as so on.
 */
@Disabled
@Autonomous(name="Debug Rotation using Encoders", group="Robot")
    public class DebugRotationWithEncoders extends LinearOpMode {

        /* Declare all motors as null */

        private DcMotor         leftFrontDrive   = null;
        private DcMotor         rightFrontDrive  = null;
        private DcMotor         rightBackDrive = null;
        private DcMotor         leftBackDrive = null;

        static final double MOTOR_TICK_COUNTS = 537.7; // goBILDA 5203 series Yellow Jacket

        // figure out how many times we need to turn the wheels to go a certain distance
        // the distance you drive with one turn of the wheel is the circumference of the wheel
        // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
        static final double WHEEL_DIAMETER_INCHES = 96 / 25.4; // in Inches
        static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches

        static final double SPEED = 0.5; // Motor Power setting

        @Override
        public void runOpMode() {
            // Initialize the drive system variables.
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
            leftBackDrive  = hardwareMap.get(DcMotor.class, "motorBackLeft");
            rightFrontDrive  = hardwareMap.get(DcMotor.class, "motorFrontRight");
            rightBackDrive  = hardwareMap.get(DcMotor.class, "motorBackRight");

            resetMotors();
            setMotorPowerToZero(); // Start by making sure the Robot is Stopped

            waitForStart();

            drive(SPEED, 2, 2, -2, -2);

        }

    private void resetMotors() {
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

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    }

    private void setMotorPowerToZero() {
        leftBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs, double rightBackRevs) {
        leftFrontDrive.setTargetPosition((int)(leftFrontRevs * MOTOR_TICK_COUNTS));
        leftBackDrive.setTargetPosition((int)(leftBackRevs * MOTOR_TICK_COUNTS));
        rightFrontDrive.setTargetPosition((int)(rightFrontRevs * MOTOR_TICK_COUNTS));
        rightBackDrive.setTargetPosition((int)(rightBackRevs * MOTOR_TICK_COUNTS));

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        int counter = 1;
        while (leftBackDrive.isBusy() || leftFrontDrive.isBusy() || rightBackDrive.isBusy() || rightFrontDrive.isBusy()) {
            telemetry.addLine("Current Position of the Motors")
                    .addData("Left Front  ", "%d", leftFrontDrive.getCurrentPosition())
                    .addData("Left Back  ", "%d", leftBackDrive.getCurrentPosition())
                    .addData("Right Front  ", "%d", rightFrontDrive.getCurrentPosition())
                    .addData("Right Back  ", "%d", rightBackDrive.getCurrentPosition())

                    .addData("Left Front Ticks ", "%d", leftFrontDrive.getTargetPosition())
                    .addData("Left Front Ticks ", "%d", leftFrontDrive.getTargetPosition())
                    .addData("Left Front Ticks ", "%d", leftFrontDrive.getTargetPosition())
                    .addData("Left Front Ticks ", "%d", leftFrontDrive.getTargetPosition())

                    .addData("Ticks front left Motor", "%.2f", leftFrontDrive.getMotorType().getTicksPerRev())
                    .addData(" Counter: ", "%d", counter++);

            telemetry.update();
        }

        sleep(1000); // wait for telemetry to update

        setMotorPowerToZero(); // Stop the Motor
    }
}

package org.firstinspires.ftc.teamcode.mmcenterstage.other;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Disabled
public class MetalMagicDriveTrain {
    private DcMotor frontLeftDrive   = null;
    private DcMotor         frontRightDrive  = null;
    private DcMotor         backLeftDrive    = null;
    private DcMotor         backRightDrive   = null;

    static final double MOTOR_TICK_COUNTS = 537.7; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER_INCHES = 96 / 25.4; // in Inches
    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches

    static final double DEGREES_MOTOR_MOVES_IN_1_REV = 45.0;

    static final double SPEED = 0.5; // Motor Power setting

    Telemetry telemetry = null;
    HardwareMap hardwareMap = null;
    /**
     * This function initializes the 4 DC Motors
     * Gets them from the hardwaremap
     * and reset it
     * @param hardwareMap
     * @param  telemetry
     */
    public void initilizeDriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
    this.telemetry = telemetry;
    this.hardwareMap = hardwareMap;

    frontLeftDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motorbackLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "motorbackRight");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
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

    private void strafe(double strafeInches) {
        // We assume that strafing right means positive
        double strafeRevs = strafeInches / CIRCUMFERENCE_INCHES;
        if (strafeInches >= 0) {
            telemetry.addData("Strafing towards right by ", "%.3f inches", strafeInches);
            telemetry.update();
            drive(SPEED,
                    1 * strafeRevs,
                    -1 * strafeRevs,
                    -1 * strafeRevs,
                    1 * strafeRevs);
        } else {
            telemetry.addData("Strafing towards Left by ", "%.3f inches", Math.abs(strafeInches));
            telemetry.update();
            drive(SPEED,
                    -1 * strafeRevs,
                    1 * strafeRevs,
                    1 * strafeRevs,
                    -1 * strafeRevs);
        }
    }

    private void moveStraightLine(double movementInInches) {
        double moveInRevs = movementInInches / CIRCUMFERENCE_INCHES;
        telemetry.addData("Moving ", "%.3f inches", movementInInches);
        telemetry.update();
        drive(SPEED, moveInRevs, moveInRevs, moveInRevs, moveInRevs);
    }

    private void rotate(double degrees) {
        // Assume positive degrees means moving towards the right
        double movementOfWheelsInRevs = Math.abs(degrees / DEGREES_MOTOR_MOVES_IN_1_REV);

        if (degrees >= 0) {
            telemetry.addData("Rotating right by ", "%.3f inches", degrees);
            telemetry.update();
            drive(SPEED,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs);
        } else {
            // Moving negative means rotating left
            telemetry.addData("Rotating left by ", "%.3finches", Math.abs(degrees));
            telemetry.update();
            drive(SPEED,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs);
        }
    }

    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs, double rightBackRevs) {

        int LFdrivetarget = (int) (leftFrontRevs * MOTOR_TICK_COUNTS) + frontLeftDrive.getCurrentPosition();
        int LBdrivetarget = (int) (leftBackRevs * MOTOR_TICK_COUNTS) + backLeftDrive.getCurrentPosition();
        int RFdrivetarget = (int) (rightFrontRevs * MOTOR_TICK_COUNTS) + frontRightDrive.getCurrentPosition();
        int RBdrivetarget = (int) (rightBackRevs * MOTOR_TICK_COUNTS) +  backRightDrive.getCurrentPosition();

        frontLeftDrive.setTargetPosition(LFdrivetarget);
        backLeftDrive.setTargetPosition(LBdrivetarget);
        frontRightDrive.setTargetPosition(RFdrivetarget);
        backRightDrive.setTargetPosition(RBdrivetarget);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        frontLeftDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backRightDrive.setPower(speed);

        while (frontLeftDrive.isBusy() || backLeftDrive.isBusy() || frontRightDrive.isBusy() || backRightDrive.isBusy()) {
            telemetry.addLine("Current Position of the Motors")
                    .addData("Left Front  ", "%d", frontLeftDrive.getCurrentPosition())
                    .addData("Left Back ", "%d", backLeftDrive.getCurrentPosition())
                    .addData("Right Front ", "%d", frontRightDrive.getCurrentPosition())
                    .addData("Right Back ", "%df", backRightDrive.getCurrentPosition());

            telemetry.addLine("Target Positions of the Motors")
                    .addData("Left Front  ", "%d", LFdrivetarget)
                    .addData("Left Back ", "%d", LBdrivetarget)
                    .addData("Right Front ", "%d", RFdrivetarget)
                    .addData("Right Back ", "%df", RBdrivetarget);

            telemetry.update();
        }
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        sleep(250);
    }

    private  static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

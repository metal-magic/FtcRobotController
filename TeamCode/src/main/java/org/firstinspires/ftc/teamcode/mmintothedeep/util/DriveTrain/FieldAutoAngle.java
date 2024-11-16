//package org.firstinspires.ftc.teamcode.mmintothedeep.util.DriveTrain;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.mmintothedeep.util.UtilityValues;
//import java.lang.Math;
//
//public class FieldAutoAngle {
//
//    // Declare drivetrain information
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
//    static final double MOTOR_TICK_COUNTS = UtilityValues.motorTicks; // goBILDA 5203 series Yellow Jacket
//    // figure out how many times we need to turn the wheels to go a certain distance
//    // the distance you drive with one turn of the wheel is the circumference of the wheel
//    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
//    public static final double WHEEL_DIAMETER_INCHES = UtilityValues.WHEEL_DIAMETER_INCHES; // in Inches
//    public static final double CIRCUMFERENCE_INCHES = UtilityValues.CIRCUMFERENCE_INCHES;
//    // pi * the diameter of the wheels in inches
//
//    public static final double DEGREES_MOTOR_MOVES_IN_1_REV = UtilityValues.DEGREES_MOTOR_MOVES_IN_1_REV;
//
//    public static final double SPEED = UtilityValues.SPEED; // Motor Power setting
//
//    HardwareMap hardwareMap = null;
//
//    public void init() {
//        this.hardwareMap = hardwareMap;
//
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");
//        // Set all the correct motor directions
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        // Set zero power behavior
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        // Initialize power to be zero
//        leftFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//    }
//
//    public void initializeIMU() {
//        // Retrieve the IMU from the hardware map
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//    }
//
//
//
//    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs, double rightBackRevs) {
//        int LFdrivetarget = (int) (leftFrontRevs * MOTOR_TICK_COUNTS) + leftFrontDrive.getCurrentPosition();
//        int LBdrivetarget = (int) (leftBackRevs * MOTOR_TICK_COUNTS) + leftBackDrive.getCurrentPosition();
//        int RFdrivetarget = (int) (rightFrontRevs * MOTOR_TICK_COUNTS) + rightFrontDrive.getCurrentPosition();
//        int RBdrivetarget = (int) (rightBackRevs * MOTOR_TICK_COUNTS) +  rightBackDrive.getCurrentPosition();
//
//        leftFrontDrive.setTargetPosition(LFdrivetarget);
//        leftBackDrive.setTargetPosition(LBdrivetarget);
//        rightFrontDrive.setTargetPosition(RFdrivetarget);
//        rightBackDrive.setTargetPosition(RBdrivetarget);
//
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        leftFrontDrive.setPower(speed);
//        leftBackDrive.setPower(speed);
//        rightFrontDrive.setPower(speed);
//        rightBackDrive.setPower(speed);
//
//        leftFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        rightBackDrive.setPower(0);
//    }
//
//}

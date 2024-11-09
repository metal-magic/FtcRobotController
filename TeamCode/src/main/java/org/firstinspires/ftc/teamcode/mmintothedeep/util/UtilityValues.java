package org.firstinspires.ftc.teamcode.mmintothedeep.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class UtilityValues {

    public static int wheelDiameter = 104; //96 for test robot
    public static int smallWheelDiameter = 104; //96 for test robot
    public static double motorTicks = 537.7; // goBILDA 5203 series Yellow Jacket
    public static String webCamName = "testWebcam";
    public static String webCamName2 = "diddyCam";

    public static boolean useWebCam1 = false;
    public static boolean useWebCam2 = false;

    public static final double MOTOR_TICK_COUNTS = motorTicks; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    public static final double WHEEL_DIAMETER_INCHES = wheelDiameter / 25.4; // in Inches
    public static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches

    public static final double DEGREES_MOTOR_MOVES_IN_1_REV = 45.0;

    public static final double SPEED = 0.7;

    public static final double LSSPEED = 0.7;

    public static final double minLS = 0;
    public static final double chamberLS = 0;
    public static final double basketLS = 0;
    public static final double maxLS = 3064;

    // for test robot
    public static final DcMotorSimple.Direction leftFrontDirection = DcMotor.Direction.FORWARD;

    public static final DcMotorSimple.Direction leftBackDirection = DcMotor.Direction.REVERSE;

    public static final DcMotorSimple.Direction rightFrontDirection = DcMotor.Direction.FORWARD;

    public static final DcMotorSimple.Direction rightBackDirection = DcMotor.Direction.FORWARD;

    // for competition robot
    public static final DcMotorSimple.Direction finalLeftFrontDirection = DcMotor.Direction.REVERSE;

    public static final DcMotorSimple.Direction finalLeftBackDirection = DcMotor.Direction.REVERSE;

    public static final DcMotorSimple.Direction finalRightFrontDirection = DcMotor.Direction.FORWARD;

    public static final DcMotorSimple.Direction finalRightBackDirection = DcMotor.Direction.FORWARD;


}
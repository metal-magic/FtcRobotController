package org.firstinspires.ftc.teamcode.mmintothedeep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class UtilityValues {

    public static int wheelDiameter = 104; //96 for test robot
    public static int smallWheelDiameter = 96; //96 for test robot
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
    public static final double COMPETITION_MOTOR_MOVES_IN_1_REV = 56.1;

    public static final double SPEED = 0.5;

    public static final double LSSPEED = 0.7;

    public static final double minLS = 0;
    public static final double chamberLS = 0;
    public static final double basketLS = 0;
    public static final double maxLS = 3064;

    public static final double offsetCamera1 = 34;

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

    // for second competition 'upper body' presets

    public static final double SLIDE_POS_DOWN = 50;
    public static final double SLIDE_POS_SPEC_DOWN = 2080;
    //changed from 2200 to 2195 - aditya 1/13
    public static final double SLIDE_POS_SPEC_UP = 2750;
    public static final double SLIDE_POS_SAMP = 5200;
    public static final double SLIDE_POS_STABLE = 2000;

    public static final double PIVOT_POS_DOWN = 0.658; //0.76-0.05;
    public static final double PIVOT_POS_HOVER = 0.57333333; //0.7-0.05;
    public static final double PIVOT_POS_SUB = 0.57333333; //0.7-0.05;
    public static final double PIVOT_POS_FLOAT = 0.30;
    public static final double PIVOT_POS_TRANSFER = 0.2105; //0.229; //0.39-0.05;
    public static final double PIVOT_POS_HANG = 0.46; //0.229; //0.39-0.05;


    public static final double PIVOT_POS_OUT_OF_SUBMERSIBLE = 0.55;

    public static final double TURN_POS_DOWN = 0.06;
    public static final double TURN_POS_TRANSFER = 0.795926;

    public static final double FLIP_POS_DOWN = 0.06; // 0.1
    public static final double FLIP_POS_SCORE = 0.7;

    public static final double GRIPPER_POS_CLOSE = 0;
    public static final double GRIPPER_POS_OPEN = 0.3;

    public static final double CLIP_POS_CLOSE = 0;
    public static final double CLIP_POS_OPEN = 0.3;

}
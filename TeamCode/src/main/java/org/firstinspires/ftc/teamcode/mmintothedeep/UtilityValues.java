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

    // for competition robot 2
    public static final DcMotorSimple.Direction finalLeftFrontDirection = DcMotor.Direction.REVERSE;

    public static final DcMotorSimple.Direction finalLeftBackDirection = DcMotor.Direction.REVERSE;

    public static final DcMotorSimple.Direction finalRightFrontDirection = DcMotor.Direction.FORWARD;

    public static final DcMotorSimple.Direction finalRightBackDirection = DcMotor.Direction.FORWARD;

    // for competition robot 3
    public static final DcMotorSimple.Direction compLeftFrontDirection = DcMotor.Direction.REVERSE;

    public static final DcMotorSimple.Direction compLeftBackDirection = DcMotor.Direction.REVERSE;

    public static final DcMotorSimple.Direction compRightFrontDirection = DcMotor.Direction.FORWARD;

    public static final DcMotorSimple.Direction compRightBackDirection = DcMotor.Direction.FORWARD;

    // for second competition 'upper body' presets

    public static final double SLIDE_POS_DOWN = 50;
    public static final double SLIDE_POS_SPEC_DOWN = 1300;
    //changed from 2200 to 2195 - aditya 1/13
    public static final double SLIDE_POS_SPEC_UP = 1984;
    public static final double SLIDE_POS_SAMP = 3384;
    public static final double SLIDE_POS_STABLE = 1400;
    public static final double SLIDE_POS_TRANSFER = 300;


    public static final double PIVOT_POS_DOWN = 0.11; //0.658; //0.76-0.05;
    public static final double PIVOT_POS_HOVER = 0.202; //0.57333333; //0.7-0.05;
    public static final double PIVOT_POS_SUB = 0.57333333; //0.7-0.05;
    public static final double PIVOT_POS_FLOAT = 0.4806;
    public static final double PIVOT_POS_TRANSFER = 0.585; //0.229; //0.39-0.05;
    public static final double PIVOT_POS_HANG = 0.30; //0.229; //0.39-0.05;
    public static final double PIVOT_POS_OUT_OF_SUBMERSIBLE = 0.2644;

    public static final double TURN_POS_DOWN = 0.0694;
    public static final double TURN_POS_TRANSFER = 0.77; //0.7178;

//    public static final double FLIP_POS_DOWN = 0.1861; //0.06; // 0.1
    public static final double FLIP_POS_DOWN = 0.13;
    public static final double FLIP_POS_SCORE = 0.8128; //0.7;
    public static final double FLIP_POS_MID = 0.36; //0.7;

    public static final double GRIPPER_POS_CLOSE = 0.11;
    public static final double GRIPPER_POS_OPEN = 0.2794;

    public static final double CLIP_POS_CLOSE = 1;
    public static final double CLIP_POS_OPEN = 0.65;

    public static final int PIVOT_MOTOR_DOWN = 0;
    public static final int PIVOT_MOTOR_ALIGN = -130;
    public static final int PIVOT_MOTOR_SUB = -200;
    public static final int PIVOT_MOTOR_FLOAT = -450;
    public static final int PIVOT_MOTOR_TRANSFER = -660;
    public static final int PIVOT_MOTOR_HANG = -275;

    public static final int PIVOT_MOTOR_DOWN_AUTO = 670;
    public static final int PIVOT_MOTOR_ALIGN_AUTO = 625;
    public static final int PIVOT_MOTOR_SUB_AUTO = 550;
    public static final int PIVOT_MOTOR_FLOAT_AUTO = 230;
    public static final int PIVOT_MOTOR_TRANSFER_AUTO = 50;


}
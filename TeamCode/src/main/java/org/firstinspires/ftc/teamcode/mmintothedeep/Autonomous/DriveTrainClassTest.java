package org.firstinspires.ftc.teamcode.mmintothedeep.Autonomous;
// TESTING THE DRIVE TRAIN CLASS AND APRIL TAG ALIGNMENT

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.util.Camera.AprilTagClass;
import org.firstinspires.ftc.teamcode.mmintothedeep.util.DriveTrain.DriveTrainFunctions;

import java.util.Date;

@Autonomous(name="Test: Drive Train Classes", group="Autonomous")
public class DriveTrainClassTest extends LinearOpMode {
    /* Declare all motors as null */
    Date currentTime = new Date();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    Servo gripperServo1 = null;
    Servo pivotServo = null;

    CRServo armMotor = null;
    static final double MOTOR_TICK_COUNTS = 537.7; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER_INCHES = 96 / 25.4; // in Inches
    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches

    static final double DEGREES_MOTOR_MOVES_IN_1_REV = 45.0;

    static final double SPEED = 0.5; // Motor Power setting

    @Override
    public void runOpMode() {
        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        pivotServo = hardwareMap.servo.get("pivotServo");
        /* Assign all the motors */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");
        armMotor = hardwareMap.crservo.get("armMotor");

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

        // ABOVE THIS, THE ENCODERS AND MOTOR ARE NOW RESET

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gripperServo1.setPosition(1);
        waitForStart();

        //Object creation for DriveTrainFunctions
        DriveTrainFunctions dtf = new DriveTrainFunctions();
        //Object creation for AprilTagClass
        AprilTagClass apriltag = new AprilTagClass();


      /*
        ============================
        THIS IS THE ACTUAL DRIVING
        ============================
       */

        dtf.moveStraightLine(5, SPEED);
        dtf.moveStraightLine(-5, SPEED);
        dtf.rotate(90, SPEED);
        dtf.rotate(-90, SPEED);
        dtf.strafe(5, SPEED);
        dtf.strafe(-5, SPEED);
        dtf.strafeAnyAngle(5, 50, SPEED);
        apriltag.alignX(-1, 1, 1);
        apriltag.alignZ(-5, 5, 1);


        //Termination
        if (currentTime.getTime() > 20000) {
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
        }

    }

}
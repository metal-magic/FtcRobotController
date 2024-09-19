package org.firstinspires.ftc.teamcode.mmintothedeep.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.util.DriveTrain.DriveTrainFunctions;

import java.util.Date;

@Autonomous(name="Blue TEST ONLY: LEFT of Gate", group="Autonomous")
public class BlueAutoLeftTest extends LinearOpMode {
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


      /*
        ============================
        THIS IS THE ACTUAL DRIVING
        ============================
       */

       /*
        METAL MAGIC CENTERSTAGE
        THIS CODE STARTS ON THE LEFT SIDE OF THE BLUE SIDE (closer to backdrop)
        STACKS PIXEL AND PARKS IN CORNER
        THIS WAS A TEST FILE TO TEST AUTONOMOUS CODE TO BE EVENTUALLY USED
        */
        //sleep lines are to avoid two lines of codes running at the same time
        gripperServo1.setPosition(1); //close claw
        sleep(250);
        dtf.moveStraightLine(24, SPEED); //drive forward to align with backboard
        dtf.rotate(90, SPEED); //rotate away the backboard so arm can move back
        dtf.moveStraightLine(-36, SPEED); //move backwards
        sleep(250);

        //next section of code moves arm backward on basis of time
        long t = System.currentTimeMillis();
        long endTimer = t + 2000;
        while (System.currentTimeMillis() < endTimer) {
            armMotor.setPower(-0.35);
        }
        armMotor.setPower(0);

        sleep(250);
        gripperServo1.setPosition(0.2); //drop pixel (open claw)
        sleep(750);

        //return arm back to standard position
        t = System.currentTimeMillis();
        endTimer = t + 2000;
        while (System.currentTimeMillis() < endTimer) {
            armMotor.setPower(+0.35);
        }

        //parking on side
        dtf.strafe(24, SPEED);
        dtf.moveStraightLine(-13, SPEED);

        //Termination
        if (currentTime.getTime() > 20000) {
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
        }

    }

}

    /*
    =====================================================
    PROGRAMMING FUNCTIONS FOR THE SEPARATE MOVEMENT TYPES
    =====================================================
     */
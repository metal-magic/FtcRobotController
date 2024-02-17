package org.firstinspires.ftc.teamcode.mmcenterstage.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Date;

@Autonomous(name="Red: RIGHT of Gate", group="Autonomous")
public class RedAutonomousRight extends LinearOpMode {
    /* Declare all motors as null */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    Servo gripperServo1 = null;
    Servo pivotServo = null;
    Servo droneServo = null;

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
        droneServo = hardwareMap.servo.get("droneServo");
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

      /*
        ============================
        THIS IS THE ACTUAL DRIVING
        ============================
       */
        droneServo.setPosition(0.10);
        gripperServo1.setPosition(1);
        sleep(AutonomousUtility.SLEEP_TIME);
        moveStraightLine(24);
        rotate(-90);
        moveStraightLine(-36);

        sleep(AutonomousUtility.SLEEP_TIME);
        long t= System.currentTimeMillis();
        long endTimer = t+2000;
        while(System.currentTimeMillis() < endTimer) {
            armMotor.setPower(-0.35);
        }
        armMotor.setPower(0);
<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mmcenterstage/autonomous/RedAutonomousRight.java

        sleep(AutonomousUtility.SLEEP_TIME);
        gripperServo1.setPosition(0.2);

        sleep(AutonomousUtility.SLEEP_TIME * 3);
=======
        sleep(250);
        gripperServo1.setPosition(0.3);
        sleep(750);
        gripperServo1.setPosition(1);
>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mmcenterstage/RedAutonomousRight.java
        t= System.currentTimeMillis();
        endTimer = t+2000;
        while(System.currentTimeMillis() < endTimer) {
            armMotor.setPower(+0.35);
        }
        strafe(-24);
        moveStraightLine(-13);


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(AutonomousUtility.SLEEP_TIME);

    }

    /*
    =====================================================
    PROGRAMMING FUNCTIONS FOR THE SEPARATE MOVEMENT TYPES
    =====================================================
     */
    private void strafe(double strafeInches) {
        // We assume that strafing right means positive
        double strafeRevs = Math.abs(strafeInches / CIRCUMFERENCE_INCHES);
        if (strafeInches >= 0) {
            telemetry.addData("Strafing towards right by ", "%.3f inches", strafeInches);

            drive(SPEED,
                    1 * strafeRevs,
                    -1 * strafeRevs,
                    -1 * strafeRevs,
                    1 * strafeRevs);
        } else {
            telemetry.addData("Strafing towards Left by ", "%.3f inches", Math.abs(strafeInches));

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


    /**
     * Function to Rotate the 4-Wheel Robot by certain amount of degrees.
     *
     * @param degrees POSITIVE degrees means rotating **RIGHT**
     */
    private void rotate(double degrees) {
        // Assume positive degrees means moving towards the right
        double movementOfWheelsInRevs = Math.abs(degrees / DEGREES_MOTOR_MOVES_IN_1_REV);

        if (degrees >= 0) {
            telemetry.addData("Rotating right by ", "%.3f inches", degrees);

            drive(SPEED,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs);
        } else {
            // Moving negative means rotating left
            telemetry.addData("Rotating left by ", "%.3finches", Math.abs(degrees));
            drive(SPEED,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs);
        }
    }

    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs, double rightBackRevs) {

        int LFdrivetarget = (int) (leftFrontRevs * MOTOR_TICK_COUNTS) + leftFrontDrive.getCurrentPosition();
        int LBdrivetarget = (int) (leftBackRevs * MOTOR_TICK_COUNTS) + leftBackDrive.getCurrentPosition();
        int RFdrivetarget = (int) (rightFrontRevs * MOTOR_TICK_COUNTS) + rightFrontDrive.getCurrentPosition();
        int RBdrivetarget = (int) (rightBackRevs * MOTOR_TICK_COUNTS) +  rightBackDrive.getCurrentPosition();

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


        sleep(AutonomousUtility.SLEEP_TIME);
    }

}
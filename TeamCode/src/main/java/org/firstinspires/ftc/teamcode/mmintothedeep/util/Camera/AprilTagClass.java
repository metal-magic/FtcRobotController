package org.firstinspires.ftc.teamcode.mmintothedeep.util.Camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mmintothedeep.util.DriveTrain.DriveTrainFunctions;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Date;

@Autonomous
public class AprilTagClass extends LinearOpMode{
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

    static final double SPEED = 1; // Motor Power setting
    @Override

    public void runOpMode() throws InterruptedException {
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        /* Assign all the motors */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");
        //armMotor = hardwareMap.crservo.get("armMotor");

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
        //gripperServo1.setPosition(1);
        waitForStart();

        //MyDriveTrain m = new MyDriveDrain();
        //m.rotate(90);


        //drawing information on the driver station camera screen
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(484.149, 484.149, 309.846, 272.681)
                .build();

        //VisionPortal myBestVisionPortal;
        //myBestVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "testWebcam"), tagProcessor);


        //stating the webcam
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .setCameraResolution(new Size(640, 480))
                .build();




        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            //alignX(-1, 1, 12);
            if (tagProcessor.getDetections().size() > 0) {
                if (tagProcessor.getDetections().get(0).ftcPose.x < (-0.5 - 1)) { //0.5 is buffer
                    strafe(2);
                }
                if (tagProcessor.getDetections().get(0).ftcPose.x > (0.5 + 1)) { //0.5 is buffer
                    strafe(-2);
                }
            }

            //rotate robot until it detects an AprilTag
            /*if ((tagProcessor.getDetections().size() != 0)&&((tagProcessor.getDetections().get(0).id % 2== 1))) {
                if (tagProcessor.getDetections().get(0).ftcPose.z <= 50) {
                    rotateRobot(0);
                } else {
                    rotateRobot(0.2);
                }
            } else {
                rotateRobot(0.2);
            }*/


            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                //sending telemetry values to the driver station
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("id", tag.id);

            }
            telemetry.update();



        }

    }

//    private void strafe(double strafeInches) {
//        // We assume that strafing right means positive
//
//        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
//        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
//        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
//        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
//        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        double strafeRevs = Math.abs(strafeInches / CIRCUMFERENCE_INCHES);
//        if (strafeInches >= 0) {
//            //telemetry.addData("Strafing towards right by ", "%.3f inches", strafeInches);
//
//            motorFrontLeft.setPower(0.3*strafeRevs);
//            motorBackLeft.setPower(-0.3*strafeRevs);
//            motorFrontRight.setPower(-0.3*strafeRevs);
//            motorBackRight.setPower(0.3*strafeRevs);
//        } else {
//            //telemetry.addData("Strafing towards Left by ", "%.3f inches", Math.abs(strafeInches));
//
//            motorFrontLeft.setPower(-0.3*strafeRevs);
//            motorBackLeft.setPower(0.3*strafeRevs);
//            motorFrontRight.setPower(0.3*strafeRevs);
//            motorBackRight.setPower(-0.3*strafeRevs);
//        }
//    }


    private void rotateRobot(double value) {
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setPower(value);
        motorBackLeft.setPower(value);
        motorFrontRight.setPower(-1 * value);
        motorBackRight.setPower(-1 * value);
    }

    public void alignX(double minX, double maxX, int myTagID) {

        //drawing information on the driver station camera screen
        AprilTagProcessor tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(484.149, 484.149, 309.846, 272.681)
                .build();


        //stating the webcam
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor1)
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .setCameraResolution(new Size(640, 480))
                .build();


        int theIndex = 6;
        for (int i = 0; i <= 5; i++) {
            if (tagProcessor1.getDetections().get(i).id == myTagID) {
                theIndex = myTagID;
                break;
            }
        }

        DriveTrainFunctions dtc = new DriveTrainFunctions();

        while (tagProcessor1.getDetections().get(theIndex).ftcPose.x < (-0.5-minX)) { //0.5 is buffer
            dtc.strafe(2, 0.5);
        }
        while (tagProcessor1.getDetections().get(theIndex).ftcPose.x > (0.5 + maxX)) { //0.5 is buffer
            dtc.strafe(2, 0.5);
        }
    }

    public void alignZ(double minZ, double maxZ, int myTagID) {

        //drawing information on the driver station camera screen
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(484.149, 484.149, 309.846, 272.681)
                .build();


        //stating the webcam
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .setCameraResolution(new Size(640, 480))
                .build();


        int theIndex = 6;
        for (int i = 0; i <= 5; i++) {
            if (tagProcessor.getDetections().get(i).id == myTagID) {
                theIndex = myTagID;
                break;
            }
        }

        DriveTrainFunctions dtc = new DriveTrainFunctions();

        while (tagProcessor.getDetections().get(theIndex).ftcPose.z < (-0.5-minZ)) { //0.5 is buffer
            dtc.moveStraightLine(0.5, 0.5);
        }
        while (tagProcessor.getDetections().get(theIndex).ftcPose.z > (0.5 + maxZ)) { //0.5 is buffer
            dtc.moveStraightLine(-0.5, 0.5);
        }
    }

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

    /*
    =====================================================
    MOVE IN STRAIGHT LINE FUNCTION
    to call:
        moveStraightLine(# of inches);
        positive # of inches -> forward
    =====================================================
    */
    private void moveStraightLine(double movementInInches) {
        double moveInRevs = movementInInches / CIRCUMFERENCE_INCHES;
        //telemetry.addData("Moving ", "%.3f inches", movementInInches);
        telemetry.update();
        drive(SPEED, moveInRevs, moveInRevs, moveInRevs, moveInRevs);
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
//            telemetry.addLine("Current Position of the Motors")
//                    .addData("Left Front  ", "%d", leftFrontDrive.getCurrentPosition())
//                    .addData("Left Back ", "%d", leftBackDrive.getCurrentPosition())
//                    .addData("Right Front ", "%d", rightFrontDrive.getCurrentPosition())
//                    .addData("Right Back ", "%df", rightBackDrive.getCurrentPosition());
//
//            telemetry.addLine("Target Positions of the Motors")
//                    .addData("Left Front  ", "%d", LFdrivetarget)
//                    .addData("Left Back ", "%d", LBdrivetarget)
//                    .addData("Right Front ", "%d", RFdrivetarget)
//                    .addData("Right Back ", "%df", RBdrivetarget);

            //telemetry.update();
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);


        sleep(250);
    }

}
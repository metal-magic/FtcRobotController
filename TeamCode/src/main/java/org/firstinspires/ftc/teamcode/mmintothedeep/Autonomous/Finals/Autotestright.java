package org.firstinspires.ftc.teamcode.mmintothedeep.Autonomous.Finals;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mmintothedeep.util.UtilityValues;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Date;
import java.util.Objects;


@Autonomous(name="Test right of Gate", group="Autonomous")
public class Autotestright extends LinearOpMode {
    Date currentTime = new Date();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public Servo gripperServo1 = null;
    public Servo pivotServo = null;
    public DcMotor linearSlideMotor = null;
    public DcMotor linearActuatorMotor = null;

    CRServo armMotor = null;
    static final double MOTOR_TICK_COUNTS = UtilityValues.motorTicks; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER_INCHES = UtilityValues.wheelDiameter / 25.4; // in Inches
    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches

    static final double DEGREES_MOTOR_MOVES_IN_1_REV = 45.0;

    static final double SPEED = UtilityValues.SPEED; // Motor Power setting

    VisionPortal visionPortal;
    VisionPortal visionPortal2;
    AprilTagProcessor tagProcessor;
    AprilTagProcessor tagProcessor2;

    @Override
    public void runOpMode() {

        initPortal();
        initMotor();
        waitForStart();


      /*
        ============================
        THIS IS THE ACTUAL DRIVING
        ============================
       */

       /*
        METAL MAGIC INTO THE DEEP
        THIS CODE STARTS ON THE LEFT SIDE OF THE BLUE SIDE (closer to backdrop)
        STACKS PIXEL AND PARKS IN CORNER
        THIS IS A TEST FILE TO TEST AUTONOMOUS CODE TO BE EVENTUALLY USED
        */
        //sleep lines are to avoid two lines of codes running at the same time
        //pivotServo.setPosition(0.6);
        //gripperServo1.setPosition(0);
        //moveStraightLine(24); //33
        strafe(0);
        linearSlideMovement(1300, false);
        strafeDiagonalLeft(15);
        //moveStraightLine(-1);
        //pivotServo.setPosition(0.635);
        linearSlideMovement(300, true);
        sleep(500);
        //gripperServo1.setPosition(0.3);
        sleep(600);
        moveStraightLine(-10);
        linearSlideMovement(50, false);
        rotate(-80);
        sleep(1000);
        //moveStraightLine(30);
        moveStraightLine(20);
        tagTelemetry(1);
        sleep(1000);
        align(-24, 24, 0, 1);
        alignToDefault("basket", 1);


        //Termination
        if (currentTime.getTime() > 20000) {
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
        }

    }

    public void alignToOffset(double x, double y, double dir, int vision) {

        double offset;
        if (vision == 1) {
            offset = UtilityValues.offsetCamera1;
            alignRotate(0, vision);
            alignX(0+offset, vision);
            alignY(y, vision);
            rotate(dir);
        }

    }

    public void alignToDefaultOffset(String s, int vision) {
        if (vision == 1) {
            if (Objects.equals(s, "basket")) {
                if (tagProcessor.getDetections().get(0).id == 11) {
                    alignToOffset(0, 70, 180, vision);
                    alignToOffset(0, 16, -45, vision); //now with tag 13
                } else if (tagProcessor.getDetections().get(0).id == 12) {
                    alignToOffset(-50, 16, 90, vision);
                    alignToOffset(0, 16, -45, vision); //now with tag 13
                } else if (tagProcessor.getDetections().get(0).id == 13) {
                    alignToOffset(0, 16, -45, vision);
                }
            }

            if (Objects.equals(s, "chamber")) {
            }
        } else if (vision == 2) {
            if (Objects.equals(s, "chamber")) {
                if (tagProcessor2.getDetections().get(0).id == 12) {
                    pivotServo.setPosition(0.6);
                    gripperServo1.setPosition(0);
                    alignY(24, vision);
                    linearSlideMovement(1300, false);
                    strafeDiagonalLeft(15);
                    //moveStraightLine(-1);
                    pivotServo.setPosition(0.635);
                    linearSlideMovement(300, true);
                    gripperServo1.setPosition(0.3);
                }
            }
        }
    }

    public void linearSlideMovement(double y, boolean maxPower) {
        double up;
        if (y > linearSlideMotor.getCurrentPosition()) {
            while (linearSlideMotor.getCurrentPosition() < y) {
                up = Math.sin(((double) (4000 - linearSlideMotor.getCurrentPosition()) / 4000) * Math.PI / 2);
                if (maxPower) {
                    linearSlideMotor.setPower(1);
                } else {
                    linearSlideMotor.setPower(up);
                }
            }
            linearSlideMotor.setPower(0);
        } else {
            while (linearSlideMotor.getCurrentPosition() > y) {
                up = Math.sin(((double) (1000+linearSlideMotor.getCurrentPosition()) /4000)*Math.PI/2);
                if (maxPower) {
                    linearSlideMotor.setPower(-1);
                } else {
                    linearSlideMotor.setPower(-1*up);
                }

            }
            linearSlideMotor.setPower(0);
        }
    }

    public void returnBackTo13Basket() {
        rotate(45);
        strafe(0);
    }

    public void alignToDefault(String s, int vision) {
        if (vision == 1) {
            if (Objects.equals(s, "basket")) {
                if (tagProcessor.getDetections().get(0).id == 11) {
                    align(0, 70, 180, vision);
                    align(0, 16, -45, vision); //now with tag 13
                } else if (tagProcessor.getDetections().get(0).id == 12) {
                    align(-50, 16, 90, vision);
                    align(0, 16, -45, vision); //now with tag 13
                } else if (tagProcessor.getDetections().get(0).id == 13) {
                    align(0, 16, -45, vision);
                }
            }

            if (Objects.equals(s, "chamber")) {
            }
        } else if (vision == 2) {
            if (Objects.equals(s, "chamber")) {
                if (tagProcessor2.getDetections().get(0).id == 12) {
                    align(0, 16, 0, 2);
                    moveStraightLine(5);
                }
            }
        }
    }

    public void alignTo(String s, int tagID, int vision) {

        if (Objects.equals(s, "basket")) {
            if (tagID == 12) {
                align(55, 16, 45, vision);
            }

        }

        if (Objects.equals(s, "chamber")) {
            if (tagID == 12) {
                align(0, 26, 180, vision);
            }
        }

        if (tagID == 13) {
            if (Objects.equals(s, "basket")) {
                alignRotate(0, vision);
                alignY(16, vision);
                alignX(-16, vision);
                alignRotate(-45, vision);

            }
        }

    }

    public void initMotor() {
        /* Assign all the motors */
        //drivetrain
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");

        //claw
        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        pivotServo = hardwareMap.servo.get("pivotServo");

        linearSlideMotor = hardwareMap.dcMotor.get("hangSlideMotor");
        linearActuatorMotor = hardwareMap.dcMotor.get("linearActuatorMotor");

        // Set all the right motor directions
        leftFrontDrive.setDirection(UtilityValues.finalLeftFrontDirection);
        leftBackDrive.setDirection(UtilityValues.finalLeftBackDirection);
        rightFrontDrive.setDirection(UtilityValues.finalRightFrontDirection);
        rightBackDrive.setDirection(UtilityValues.finalRightBackDirection);


        // Reset encoders positions
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideMotor.setDirection(CRServo.Direction.FORWARD);

        /*while (hangSlideMotor.getCurrentPosition() > 0) {
            hangSlideMotor.setPower(-0.5);
        }
        while (hangSlideMotor.getCurrentPosition() < 0) {
            hangSlideMotor.setPower(0.3);
        }*/

        ((ServoImplEx) pivotServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripperServo1.setPosition(0);
        pivotServo.setPosition(0);
        gripperServo1.setPosition(0);
        pivotServo.setPosition(0);

        // ABOVE THIS, THE ENCODERS AND MOTOR ARE NOW RESET

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gripperServo1.setPosition(0);
        pivotServo.setPosition(0.48);

        linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initPortal() {

        // Because we want to show two camera feeds simultaneously, we need to inform
        // the SDK that we want it to split the camera monitor area into two smaller
        // areas for us. It will then give us View IDs which we can pass to the individual
        // vision portals to allow them to properly hook into the UI in tandem.
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

        // We extract the two view IDs from the array to make our lives a little easier later.
        // NB: the array is 2 long because we asked for 2 portals up above.
        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];

        //drawing information on the driver station camera screen
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(484.149, 484.149, 309.846, 272.681)
                .build();

        tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(513.474, 513.474, 316.919, 249.760)
                .build();

        //stating the webcam
        visionPortal = new VisionPortal.Builder()
                .setLiveViewContainerId(portal1ViewId)
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .setCameraResolution(new Size(640, 480))
                .build();

        visionPortal2 = new VisionPortal.Builder()
                .setLiveViewContainerId(portal2ViewId)
                .addProcessor(tagProcessor2)
                .setCamera(hardwareMap.get(WebcamName.class, "diddyCam"))
                .setCameraResolution(new Size(640, 480))
                .build();

    }

    public void tagTelemetry(int vision) {
        telemetry.addData("Vision portal: ", vision);
        if (vision == 1) {
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
        } else if (vision == 2) {
            if (tagProcessor2.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor2.getDetections().get(0);
                //sending telemetry values to the driver station
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("id", tag.id);
            }
        }
        telemetry.update();
    }

    public void align(int x, int y, int dir, int vision) {
        alignRotate(0, vision);
        alignY(y, vision);
        alignX(x, vision);
        rotate(dir);
    }

    public void alignRotate(int dir, int vision) {

        double rotateNew;
        double originalY;
        double rotateRadians;
        double correctX;

        if (vision == 1) {
            if (tagProcessor.getDetections().size() > 0) {
                rotateNew = tagProcessor.getDetections().get(0).ftcPose.yaw - dir;
                originalY = tagProcessor.getDetections().get(0).ftcPose.y;

                if (tagProcessor.getDetections().get(0).ftcPose.yaw < (-0.5 + dir)) { //0.5 is buffer
                    //strafe(1);
                    rotate(-rotateNew);
                }
                else if (tagProcessor.getDetections().get(0).ftcPose.yaw > (0.5 + dir)) { //0.5 is buffer
                    //strafe(-1);
                    rotate(-rotateNew);
                }

                rotateRadians = Math.toRadians(rotateNew);
                correctX = Math.tan(rotateRadians) * originalY;
                strafe(-1*correctX);

            }
        } else if (vision == 2) {
            if (tagProcessor2.getDetections().size() > 0) {
                rotateNew = tagProcessor2.getDetections().get(0).ftcPose.yaw - dir;
                originalY = tagProcessor2.getDetections().get(0).ftcPose.y;

                if (tagProcessor2.getDetections().get(0).ftcPose.yaw < (-0.5 + dir)) { //0.5 is buffer
                    //strafe(1);
                    rotate(-rotateNew);
                }
                if (tagProcessor2.getDetections().get(0).ftcPose.yaw > (0.5 + dir)) { //0.5 is buffer
                    //strafe(-1);
                    rotate(-rotateNew);
                }

                rotateRadians = Math.toRadians(rotateNew);
                correctX = Math.tan(rotateRadians) * originalY;
                strafe(correctX);
            }
        }

    }

    public void alignX(double x, int vision) {

        double xPosNew;
        //alignX(-1, 1, 12);
        if (vision == 1) {
            if (tagProcessor.getDetections().size() > 0) {
                xPosNew = tagProcessor.getDetections().get(0).ftcPose.x - x;

                if (tagProcessor.getDetections().get(0).ftcPose.x < (-0.5 + x)) { //0.5 is buffer
                    //strafe(1);
                    strafe(1 * xPosNew);
                }
                if (tagProcessor.getDetections().get(0).ftcPose.x > (0.5 + x)) { //0.5 is buffer
                    //strafe(-1);
                    strafe(1 * xPosNew);
                }
            }
        } else if (vision == 2) {
            if (tagProcessor2.getDetections().size() > 0) {
                xPosNew = tagProcessor2.getDetections().get(0).ftcPose.x - x;

                if (tagProcessor2.getDetections().get(0).ftcPose.x < (-0.5 + x)) { //0.5 is buffer
                    //strafe(1);
                    strafe(-1 * xPosNew);
                }
                if (tagProcessor2.getDetections().get(0).ftcPose.x > (0.5 + x)) { //0.5 is buffer
                    //strafe(-1);
                    strafe(-1 * xPosNew);
                }
            }
        }
    }

    public void alignY(double y, int vision) {
        double yPosNew;
        //double moveInRevs;
        //alignX(-1, 1, 12);
        if (vision == 1) {
            if (tagProcessor.getDetections().size() > 0) {
                yPosNew = tagProcessor.getDetections().get(0).ftcPose.y - y;
                //moveInRevs = yPosNew / CIRCUMFERENCE_INCHES;

                if (tagProcessor.getDetections().get(0).ftcPose.y < (-0.5 + y)) { //0.5 is buffer
                    //strafe(1);
                    moveStraightLine(1 * yPosNew);
                }
                if (tagProcessor.getDetections().get(0).ftcPose.y > (0.5 + y)) { //0.5 is buffer
                    //strafe(-1);
                    moveStraightLine(1 * yPosNew);
                }
            }
        } else if (vision == 2) {
            if (tagProcessor2.getDetections().size() > 0) {
                yPosNew = tagProcessor2.getDetections().get(0).ftcPose.y - y;
                //moveInRevs = yPosNew / CIRCUMFERENCE_INCHES;

                if (tagProcessor2.getDetections().get(0).ftcPose.y < (-0.5 + y)) { //0.5 is buffer
                    //strafe(1);
                    moveStraightLine(-1 * yPosNew);
                }
                if (tagProcessor2.getDetections().get(0).ftcPose.y > (0.5 + y)) { //0.5 is buffer
                    //strafe(-1);
                    moveStraightLine(-1 * yPosNew);
                }
            }
        }
    }

    public void moveLinearSlideRevs(double y) {
        double up;
        if (y > 0) {
            while (linearSlideMotor.getCurrentPosition() < 3064 && linearSlideMotor.getCurrentPosition() < y) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //hangSlideMotor.setPower(1* UtilityValues.LSSPEED);
                up = Math.sin(((double) (4000 - linearSlideMotor.getCurrentPosition()) / 4000) * Math.PI / 2);
                linearSlideMotor.setPower(/*UtilityValues.LSSPEED * */up*gamepad2.right_trigger);
            }
            while (linearSlideMotor.getCurrentPosition() > 3064) {
                linearSlideMotor.setPower(-0.3);
            }
            linearSlideMotor.setPower(0);
        } else if (y < 0) {
            while (linearSlideMotor.getCurrentPosition() > 0 && linearSlideMotor.getCurrentPosition() > y) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ///hangSlideMotor.setPower(-1*UtilityValues.LSSPEED);
                up = Math.sin(((double) (1000+linearSlideMotor.getCurrentPosition()) /4000)*Math.PI/2);
                linearSlideMotor.setPower(-1* /*UtilityValues.LSSPEED**/up*gamepad2.left_trigger);
            }
            while (linearSlideMotor.getCurrentPosition() < 0) {
                linearSlideMotor.setPower(-0.3);
            }
            linearSlideMotor.setPower(0);
        }
    }

    public void moveLinearSlide(double inches) {
        double inchesWithoutRobotHeight = inches - 3;
        if (inchesWithoutRobotHeight < 0) {
            inchesWithoutRobotHeight = 0;
        }
        //double mm = inches * 25.4;
        double y; // = mm * (984.0 / 3064.0);
        y = inchesWithoutRobotHeight * (3064.0 / 40.5);
        double up;
        if (y > 0) {
            while (linearSlideMotor.getCurrentPosition() < 3064 && linearSlideMotor.getCurrentPosition() < y) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //hangSlideMotor.setPower(1* UtilityValues.LSSPEED);
                up = Math.sin(((double) (4000 - linearSlideMotor.getCurrentPosition()) / 4000) * Math.PI / 2);
                linearSlideMotor.setPower(/*UtilityValues.LSSPEED * */up*gamepad2.right_trigger);
            }
            while (linearSlideMotor.getCurrentPosition() > 3064) {
                linearSlideMotor.setPower(-0.3);
            }
            linearSlideMotor.setPower(0);
        } else if (y < 0) {
            while (linearSlideMotor.getCurrentPosition() > 0 && linearSlideMotor.getCurrentPosition() > y) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ///hangSlideMotor.setPower(-1*UtilityValues.LSSPEED);
                up = Math.sin(((double) (1000+linearSlideMotor.getCurrentPosition()) /4000)*Math.PI/2);
                linearSlideMotor.setPower(-1* /*UtilityValues.LSSPEED**/up*gamepad2.left_trigger);
            }
            while (linearSlideMotor.getCurrentPosition() < 0) {
                linearSlideMotor.setPower(-0.3);
            }
            linearSlideMotor.setPower(0);
        }
    }

    public void rotate(double degrees) {

        double robotSpeed = SPEED;
        // Assume positive degrees means moving towards the right
        double movementOfWheelsInRevs = Math.abs(degrees / DEGREES_MOTOR_MOVES_IN_1_REV);

        if (degrees >= 0) {
            drive(robotSpeed,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs);
        } else {
            // Moving negative means rotating left
            drive(robotSpeed,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs);
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
        telemetry.addData("Moving ", "%.3f inches", movementInInches);
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

    public void strafeDiagonalLeft(double strafeLeftInches) {
        double robotSpeed = SPEED;

        double strafeLeftRevs = Math.abs(strafeLeftInches / CIRCUMFERENCE_INCHES);

        if (strafeLeftInches >= 0) {
            drive(robotSpeed,
                    0,
                    1 * strafeLeftRevs,
                    1 * strafeLeftRevs,
                    0);
        } else {
            drive(robotSpeed,
                    0,
                    -1 * strafeLeftRevs,
                    -1 * strafeLeftRevs,
                    0);
        }
    }

    public void strafeDiagonalRight(double strafeLeftInches) {

        double robotSpeed = SPEED;
        double strafeLeftRevs = Math.abs(strafeLeftInches / CIRCUMFERENCE_INCHES);

        if (strafeLeftInches >= 0) {
            drive(robotSpeed,
                    1 * strafeLeftRevs,
                    0,
                    0,
                    1 * strafeLeftRevs);
        } else {
            drive(robotSpeed,
                    -1 * strafeLeftRevs,
                    0,
                    0,
                    -1 * strafeLeftRevs);
        }

    }

}
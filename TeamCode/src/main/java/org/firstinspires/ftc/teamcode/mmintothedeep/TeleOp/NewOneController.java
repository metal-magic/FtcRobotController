/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 *] of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mmcenterstage.HardwareTesting.LeftStrafeTest;
import org.firstinspires.ftc.teamcode.mmcenterstage.other.OldSensorColor2;
import org.firstinspires.ftc.teamcode.mmintothedeep.util.UtilityValues;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Date;
import java.util.Objects;
import java.util.concurrent.TimeUnit;
import java.util.Timer;

/*
  =========================================
  This OpMode was preserved because Om
  wanted his fine controls to be controlled
  with the trigger
  =========================================
 */

@TeleOp(name = "NEW ONE CONTROLLER TeleOp")
public class NewOneController extends OpMode {

    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public Servo gripperServo1 = null;
    // public Servo gripperServo2 = null;
    public Servo pivotServo = null;
    // public Servo fakeServo = null;
    // public Servo fakeServo2 = null;

    // public Servo droneServo = null;

    public DcMotor linearSlideMotor = null;
    public DcMotor hangSlideMotor = null;
    public DcMotor hangSlideMotor2 = null;

    boolean isPressed = false;
    boolean isPressed2 = false;

    boolean isPresetPressed = false;
    boolean wasPresetPressed = false;
    boolean swPreset = false;

    boolean isPressedEndOHYE = false;
    // public DcMotor linearActuatorMotor = null;

    public Date previousTime = new Date();

    public float armSpeedCounter = 0;
    // TouchSensor touchSensor = null;
    // OldSensorColor2 board = new OldSensorColor2();

    public static float setPositionCounter = 0;

    static final double MOTOR_TICK_COUNTS = UtilityValues.motorTicks; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the
    // wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER_INCHES = UtilityValues.wheelDiameter / 25.4; // in Inches
    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in
                                                                                // inches

    static final double DEGREES_MOTOR_MOVES_IN_1_REV = 45.0;

    static final double SPEED = UtilityValues.SPEED; // Motor Power setting

    VisionPortal visionPortal;
    VisionPortal visionPortal2;
    AprilTagProcessor tagProcessor;
    AprilTagProcessor tagProcessor2;

    // private static ElapsedTime stopWatch = new ElapsedTime();

    long start = System.currentTimeMillis();

    boolean moveSlideUp = false;
    boolean moveSlideDown = false;

    @Override
    public void init() {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        // gripperServo2 = hardwareMap.servo.get("gripperServo2");
        pivotServo = hardwareMap.servo.get("pivotServo");
        // fakeServo = hardwareMap.servo.get("fakeServo");
        // fakeServo2 = hardwareMap.servo.get("fakeServo2");

        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        hangSlideMotor = hardwareMap.dcMotor.get("hangSlideMotor1");
        hangSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangSlideMotor2 = hardwareMap.dcMotor.get("hangSlideMotor2");
        hangSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // linearActuatorMotor = hardwareMap.dcMotor.get("linearActuatorMotor");

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // linearActuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // linearActuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripperServo1.setPosition(0);
        pivotServo.setPosition(0.673);

        hangSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangSlideMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        initPortal();

    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up) {
            moveSlideUp = true;
        }
        if (gamepad2.dpad_right) {
            moveSlideDown = true;
        }
        if (gamepad2.dpad_left) {
            moveSlideDown = true;
        }
        if (moveSlideDown) {
            moveSlideUp = false;
        }
        if (moveSlideUp) {
            moveSlideDown = false;
        }

        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
         * GAMEPAD 2 IS MAIN AND GAMEPAD 1 IS HANGING
         * 
         */
        double y = -gamepad2.left_stick_y + gamepad1.left_stick_y; // REVERSED -gamepad1.left_stick_y.gamestick so
        // gamepad1 can also do movement for hanging
        // making sure it doesnt go over 1 or -1
        if (y < -1) {
            y = -1;
        } else if (y > 1) {
            y = 1;
        }
        double x = gamepad2.left_stick_x - gamepad1.left_stick_x; // gamepad1 can also do movement for hanging
        // making sure it doesnt go over 1 or -1
        if (x > 1) {
            x = 1;
        } else if (x < -1) {
            x = -1;
        }
        double rx = gamepad2.right_stick_x + gamepad1.right_stick_x; // gamepad1 can also do movement for hanging
        // making sure it doesnt go over 1 or -1
        if (rx > 1) {
            rx = 1;
        } else if (rx < -1) {
            rx = -1;
        }

        // Denominator is the largest motor power (abs value) or 1
        // This makes sure that the ratio stays the same
        // but only when at least one is out of range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        boolean CutPower = false;
        double motorSpeed;
        // fakeServo.setPosition(1);
        // fakeServo2.setPosition(1);

        telemetry.addData("Cutpower", CutPower);
        if (gamepad1.back && !CutPower) {
            // Button cuts all power except linear slides/actuators
            CutPower = true;
        }
        if (gamepad1.back && CutPower) {
            CutPower = false;
        }

        if (!CutPower) {
            telemetry.addData("slide", linearSlideMotor.getCurrentPosition());

            if (gamepad1.right_trigger >= 0.3F) {
                // Fine controls
                motorSpeed = 0.20;
            } else {
                // Reg speed
                motorSpeed = 0.75;
            }

            motorFrontLeft.setPower(frontLeftPower * motorSpeed);
            motorBackLeft.setPower(backLeftPower * motorSpeed);
            motorFrontRight.setPower(frontRightPower * motorSpeed);
            motorBackRight.setPower(backRightPower * motorSpeed);

            isPresetPressed = gamepad1.b;

            if (isPresetPressed && !wasPresetPressed) {
                swPreset = !swPreset;
            }

            wasPresetPressed = gamepad1.b;

            if (gamepad2.left_bumper) {
                gripperServo1.setPosition(0.15);
            } else if (gamepad2.right_bumper) {
                gripperServo1.setPosition(0);
            }

            if (gamepad2.y) {
                pivotServo.setPosition(pivotServo.getPosition() + 0.009);
            } else if (gamepad2.a) {
                if (pivotServo.getPosition() < 0.09) {
                    pivotServo.setPosition(0.10);
                } else {
                    pivotServo.setPosition(pivotServo.getPosition() - 0.009);
                }
            } else if (gamepad2.dpad_up) {
                pivotServo.setPosition(0.4661);
            } else if (gamepad2.dpad_down) {
                pivotServo.setPosition(0.115);
            } else if (gamepad2.dpad_right) {
                pivotServo.setPosition(0.225);
            } else if (gamepad2.dpad_left) {
                pivotServo.setPosition(0.31);
            }
        }

        if (linearSlideMotor.getCurrentPosition() < 4000 && gamepad2.right_trigger >= 0.1F) {
            linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(1);
        } else if (linearSlideMotor.getCurrentPosition() > 50 && gamepad2.left_trigger >= 0.1F) {
            linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(-1);
        } else if (moveSlideUp) {
            if (linearSlideMotor.getCurrentPosition() < 4000) {
                linearSlideMotor.setPower(0.7);
            } else {
                linearSlideMotor.setPower(0);
                moveSlideUp = false;
            }
        } else if (moveSlideDown) {
            if (linearSlideMotor.getCurrentPosition() > 10) {
                linearSlideMotor.setPower(-0.7);
            } else {
                pivotServo.setPosition(0.225);
                linearSlideMotor.setPower(0);
                moveSlideDown = false;
            }
        } else {
            linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(0);
        }

        if (gamepad1.dpad_left) {
            isPressed = false;
        }

        if (gamepad1.right_bumper) {
            hangSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            hangSlideMotor.setPower(0.7);
        } else if (gamepad1.left_bumper) {
            if (hangSlideMotor.getCurrentPosition() > -4338) {
                hangSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hangSlideMotor.setPower(-0.7);
            }
        } else {
            if (!isPressedEndOHYE) {
                hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hangSlideMotor.setPower(0);
            }
        }

        if (gamepad1.right_trigger >= 0.3F) {
            hangSlideMotor2.setDirection(DcMotor.Direction.FORWARD);
            hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangSlideMotor2.setPower(-0.7 * 0.41);
        } else if (gamepad1.left_trigger >= 0.3F) {
            if (hangSlideMotor.getCurrentPosition() > -4338) {
                hangSlideMotor2.setDirection(DcMotor.Direction.FORWARD);
                hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hangSlideMotor2.setPower(0.7 * 0.41);
            }
        } else {
            if (!isPressedEndOHYE) {
                hangSlideMotor2.setPower(0);
            }
        }

        if (gamepad1.dpad_up) {
            isPressedEndOHYE = true;
            pivotServo.setPosition(0.31);
        } else {
            if (isPressedEndOHYE) {
                hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hangSlideMotor.setPower(-0.7);
                hangSlideMotor2.setPower(0.7 * 0.41);
            }
        }

        telemetry.addData("hang Slide position, ", hangSlideMotor.getCurrentPosition());
        telemetry.addData("hang Slide2 position", hangSlideMotor2.getCurrentPosition());

        telemetry.addData("Claw Position,", gripperServo1.getPosition());
        // telemetry.addData("Linear Slide Position",
        // hangSlideMotor.getCurrentPosition());
        // telemetry.addData("Linear Slide Speed", hangSlideMotor.getPower());
        // telemetry.addData("Linear Actuator Position",
        // linearActuatorMotor.getCurrentPosition());
        // telemetry.addData("Linear Actuator Speed", linearActuatorMotor.getPower());
        telemetry.addData("Claw Join Position,", pivotServo.getPosition());
        telemetry.addData("slide", linearSlideMotor.getCurrentPosition());
        telemetry.addData("Hanging slide 1", hangSlideMotor.getCurrentPosition());
        telemetry.addData("Hanging slide 2", hangSlideMotor2.getCurrentPosition());

        telemetry.update();

    }

    public void alignToDefault(String s, int vision) {
        if (vision == 1) {
            if (Objects.equals(s, "basket")) {
                if (tagProcessor.getDetections().get(0).id == 11) {
                    align(0, 70, 180, vision);
                    align(0, 16, -45, vision); // now with tag 13
                } else if (tagProcessor.getDetections().get(0).id == 12) {
                    align(-50, 16, 90, vision);
                    align(0, 16, -45, vision); // now with tag 13
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

    public void initPortal() {

        // Because we want to show two camera feeds simultaneously, we need to inform
        // the SDK that we want it to split the camera monitor area into two smaller
        // areas for us. It will then give us View IDs which we can pass to the
        // individual
        // vision portals to allow them to properly hook into the UI in tandem.
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

        // We extract the two view IDs from the array to make our lives a little easier
        // later.
        // NB: the array is 2 long because we asked for 2 portals up above.
        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];

        // drawing information on the driver station camera screen
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

        // stating the webcam
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
                // sending telemetry values to the driver station
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
                // sending telemetry values to the driver station
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("id", tag.id);
            }
        }
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

                if (tagProcessor.getDetections().get(0).ftcPose.yaw < (-0.5 + dir)) { // 0.5 is buffer
                    // strafe(1);
                    rotate(-rotateNew);
                } else if (tagProcessor.getDetections().get(0).ftcPose.yaw > (0.5 + dir)) { // 0.5 is buffer
                    // strafe(-1);
                    rotate(-rotateNew);
                }

                rotateRadians = Math.toRadians(rotateNew);
                correctX = Math.tan(rotateRadians) * originalY;
                strafe(1 * correctX);

            }
        } else if (vision == 2) {
            if (tagProcessor2.getDetections().size() > 0) {
                rotateNew = tagProcessor2.getDetections().get(0).ftcPose.yaw - dir;
                originalY = tagProcessor2.getDetections().get(0).ftcPose.y;

                if (tagProcessor2.getDetections().get(0).ftcPose.yaw < (-0.5 + dir)) { // 0.5 is buffer
                    // strafe(1);
                    rotate(-rotateNew);
                }
                if (tagProcessor2.getDetections().get(0).ftcPose.yaw > (0.5 + dir)) { // 0.5 is buffer
                    // strafe(-1);
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
        // alignX(-1, 1, 12);
        if (vision == 1) {
            if (tagProcessor.getDetections().size() > 0) {
                xPosNew = tagProcessor.getDetections().get(0).ftcPose.x - x;

                if (tagProcessor.getDetections().get(0).ftcPose.x < (-0.5 + x)) { // 0.5 is buffer
                    // strafe(1);
                    strafe(1 * xPosNew);
                }
                if (tagProcessor.getDetections().get(0).ftcPose.x > (0.5 + x)) { // 0.5 is buffer
                    // strafe(-1);
                    strafe(1 * xPosNew);
                }
            }
        } else if (vision == 2) {
            if (tagProcessor2.getDetections().size() > 0) {
                xPosNew = tagProcessor2.getDetections().get(0).ftcPose.x - x;

                if (tagProcessor2.getDetections().get(0).ftcPose.x < (-0.5 + x)) { // 0.5 is buffer
                    // strafe(1);
                    strafe(-1 * xPosNew);
                }
                if (tagProcessor2.getDetections().get(0).ftcPose.x > (0.5 + x)) { // 0.5 is buffer
                    // strafe(-1);
                    strafe(-1 * xPosNew);
                }
            }
        }
    }

    public void alignY(double y, int vision) {
        double yPosNew;
        // double moveInRevs;
        // alignX(-1, 1, 12);
        if (vision == 1) {
            if (tagProcessor.getDetections().size() > 0) {
                yPosNew = tagProcessor.getDetections().get(0).ftcPose.y - y;
                // moveInRevs = yPosNew / CIRCUMFERENCE_INCHES;

                if (tagProcessor.getDetections().get(0).ftcPose.y < (-0.5 + y)) { // 0.5 is buffer
                    // strafe(1);
                    moveStraightLine(1 * yPosNew);
                }
                if (tagProcessor.getDetections().get(0).ftcPose.y > (0.5 + y)) { // 0.5 is buffer
                    // strafe(-1);
                    moveStraightLine(1 * yPosNew);
                }
            }
        } else if (vision == 2) {
            if (tagProcessor2.getDetections().size() > 0) {
                yPosNew = tagProcessor2.getDetections().get(0).ftcPose.y - y;
                // moveInRevs = yPosNew / CIRCUMFERENCE_INCHES;

                if (tagProcessor2.getDetections().get(0).ftcPose.y < (-0.5 + y)) { // 0.5 is buffer
                    // strafe(1);
                    moveStraightLine(-1 * yPosNew);
                }
                if (tagProcessor2.getDetections().get(0).ftcPose.y > (0.5 + y)) { // 0.5 is buffer
                    // strafe(-1);
                    moveStraightLine(-1 * yPosNew);
                }
            }
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
     * =====================================================
     * MOVE IN STRAIGHT LINE FUNCTION
     * to call:
     * moveStraightLine(# of inches);
     * positive # of inches -> forward
     * =====================================================
     */
    private void moveStraightLine(double movementInInches) {
        double moveInRevs = movementInInches / CIRCUMFERENCE_INCHES;
        telemetry.addData("Moving ", "%.3f inches", movementInInches);
        telemetry.update();
        drive(SPEED, moveInRevs, moveInRevs, moveInRevs, moveInRevs);
    }

    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs,
            double rightBackRevs) {

        int LFdrivetarget = (int) (leftFrontRevs * MOTOR_TICK_COUNTS) + leftFrontDrive.getCurrentPosition();
        int LBdrivetarget = (int) (leftBackRevs * MOTOR_TICK_COUNTS) + leftBackDrive.getCurrentPosition();
        int RFdrivetarget = (int) (rightFrontRevs * MOTOR_TICK_COUNTS) + rightFrontDrive.getCurrentPosition();
        int RBdrivetarget = (int) (rightBackRevs * MOTOR_TICK_COUNTS) + rightBackDrive.getCurrentPosition();

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

        while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy()
                || rightBackDrive.isBusy()) {
            // telemetry.addLine("Current Position of the Motors")
            // .addData("Left Front ", "%d", leftFrontDrive.getCurrentPosition())
            // .addData("Left Back ", "%d", leftBackDrive.getCurrentPosition())
            // .addData("Right Front ", "%d", rightFrontDrive.getCurrentPosition())
            // .addData("Right Back ", "%df", rightBackDrive.getCurrentPosition());
            //
            // telemetry.addLine("Target Positions of the Motors")
            // .addData("Left Front ", "%d", LFdrivetarget)
            // .addData("Left Back ", "%d", LBdrivetarget)
            // .addData("Right Front ", "%d", RFdrivetarget)
            // .addData("Right Back ", "%df", RBdrivetarget);

            // telemetry.update();
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

}
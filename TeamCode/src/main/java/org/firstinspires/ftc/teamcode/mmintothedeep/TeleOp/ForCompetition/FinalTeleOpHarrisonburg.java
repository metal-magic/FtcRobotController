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

package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.ForCompetition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

import java.util.Date;

/*
  =========================================
  This OpMode was created to test the linear slide motor
  =========================================
 */

@TeleOp(name = "!! FINAL TeleOp for Harrisonburg competition")
public class FinalTeleOpHarrisonburg extends LinearOpMode {

    public Servo gripperServo1 = null;
    // public Servo gripperServo2 = null;
    public Servo pivotServo = null;
    public Servo turnServo = null;
    public Servo clipServo = null;
    public Servo flipServo = null;
    public DcMotor linearSlideMotor = null;
    DcMotor leftFrontDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightBackDrive = null;

    public DcMotor hangSlideMotor = null;
    public DcMotor hangSlideMotor2 = null;

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

    boolean intake = false;
    long setTime = 0;

    boolean slideUp = false;
    boolean slideDown = false;

    static final double slidePosDown = UtilityValues.SLIDE_POS_DOWN;
    static final double slidePosSpecDown = UtilityValues.SLIDE_POS_SPEC_DOWN;
    static final double slidePosSpecUp = UtilityValues.SLIDE_POS_SPEC_UP;
    static final double slidePosUp = UtilityValues.SLIDE_POS_SAMP;
    static final double slidePosStable = UtilityValues.SLIDE_POS_STABLE;

    static final double pivotPosDown = UtilityValues.PIVOT_POS_DOWN;
    static final double pivotPosHover = UtilityValues.PIVOT_POS_HOVER;
    static final double pivotPosFloat = UtilityValues.PIVOT_POS_FLOAT;
    static final double pivotPosTransfer = UtilityValues.PIVOT_POS_TRANSFER;

    static final double turnPosDown = UtilityValues.TURN_POS_DOWN;
    static final double turnPosTransfer = UtilityValues.TURN_POS_TRANSFER;

    static final double flipPosDown = UtilityValues.FLIP_POS_DOWN;
    static final double flipPosScore = UtilityValues.FLIP_POS_SCORE;

    static final double gripperPosClose = UtilityValues.GRIPPER_POS_CLOSE;
    static final double gripperPosOpen = UtilityValues.GRIPPER_POS_OPEN;

    static final double clipPosClose = UtilityValues.CLIP_POS_CLOSE;
    static final double clipPosOpen = UtilityValues.CLIP_POS_OPEN;

    boolean slideMidUp = false;
    boolean slideMidDown = false;
    boolean slideStable = false;

    boolean leftBumper_isPressed = false;
    boolean leftBumper_wasPressed = false;
    int currGripperPos = 0;

    boolean rightBumper_isPressed = false;
    boolean rightBumper_wasPressed = false;
    int currSpecPos = 0;

    boolean isTransferring = false;

    long startTime = 0;

    boolean isPressedEndOHYE = false;

    public void runOpMode() {

        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        // gripperServo2 = hardwareMap.servo.get("gripperServo2");
        pivotServo = hardwareMap.servo.get("pivotServo");
        pivotServo.setDirection(Servo.Direction.REVERSE);

        turnServo = hardwareMap.servo.get("turnServo");
        //gripperServo1.setDirection(Servo.Direction.REVERSE);

        clipServo = hardwareMap.servo.get("clipServo");

        flipServo = hardwareMap.servo.get("flipServo");

        hangSlideMotor = hardwareMap.dcMotor.get("hangSlideMotor1");
        hangSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hangSlideMotor2 = hardwareMap.dcMotor.get("hangSlideMotor2");
        hangSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        hangSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangSlideMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        pivotServo.setPosition(UtilityValues.PIVOT_POS_HOVER);
        turnServo.setPosition(UtilityValues.TURN_POS_DOWN);
        gripperServo1.setPosition(gripperPosOpen);

        double startTimeAtStart = System.currentTimeMillis();
        while (linearSlideMotor.getCurrentPosition() > 50) {
            if (System.currentTimeMillis() > startTimeAtStart + 2000) {
                break;
            }
            linearSlideMotor.setPower(-1);
        }
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flipServo.setPosition(flipPosDown);

        boolean CutPower = false;
        double motorSpeed;
        while (opModeIsActive()) {

            double y = -gamepad2.left_stick_y - gamepad1.left_stick_y / 2; // REVERSED -gamepad1.left_stick_y.gamestick so
            // gamepad1 can also do movement for hanging
            // making sure it doesnt go over 1 or -1
            if (y < -1) {
                y = -1;
            } else if (y > 1) {
                y = 1;
            }
            double x = gamepad2.left_stick_x + gamepad1.left_stick_x / 2; // gamepad1 can also do movement for hanging
            // making sure it doesnt go over 1 or -1
            if (x > 1) {
                x = 1;
            } else if (x < -1) {
                x = -1;
            }
            double rx = gamepad2.right_stick_x + gamepad1.right_stick_x / 2; // gamepad1 can also do movement for hanging
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
            motorSpeed = 1;

            leftFrontDrive.setPower(frontLeftPower * motorSpeed);
            leftBackDrive.setPower(backLeftPower * motorSpeed);
            rightFrontDrive.setPower(frontRightPower * motorSpeed);
            rightBackDrive.setPower(backRightPower * motorSpeed);

            if (gamepad2.right_trigger >= 0.3F) {
                motorSpeed = 0.3;
            }

            // Manual control

            if (linearSlideMotor.getCurrentPosition() < 5350 && gamepad1.a) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlideMotor.setPower(0.8);
            } else if (linearSlideMotor.getCurrentPosition() > 50 && gamepad1.b) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlideMotor.setPower(-0.8);
            } else {
                if (!slideUp && !slideDown && !slideMidUp && !slideMidDown && !slideStable) {
                    linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    linearSlideMotor.setPower(0);
                }
            }


            // Transfer and slide up
            if (gamepad2.dpad_up) {
                if (linearSlideMotor.getCurrentPosition() > slidePosDown) {
                    linearSlideMotor.setPower(-1);
                }
                isTransferring = true;
                flipServo.setPosition(flipPosDown);
                gripperServo1.setPosition(gripperPosClose);
                pivotServo.setPosition(pivotPosTransfer);
                // nah dontsleep(500);
                turnServo.setPosition(turnPosTransfer);
                startTime = System.currentTimeMillis();
//                sleep(800);
//                gripperServo1.setPosition(gripperPosOpen);
//                sleep(500);
//                pivotServo.setPosition(pivotPosFloat);
//                sleep(500);
//                slideUp = false;
//                slideDown = false;
//                slideMidUp = false;
//                slideMidDown = false;
//                slideStable = true;
            }

            if (isTransferring) {
                if (System.currentTimeMillis() > startTime + 2000.0) {
                    slideUp = false;
                    slideDown = false;
                    slideMidUp = false;
                    slideMidDown = false;
                    slideStable = true;
                    isTransferring = false;
                } else if (System.currentTimeMillis() > startTime + 1500.0) {
                    pivotServo.setPosition(pivotPosFloat);
                } else if (System.currentTimeMillis() > startTime + 1000.0) {
                    gripperServo1.setPosition(gripperPosOpen);
                }
            }

            // align position
            if (gamepad2.dpad_right) {
                turnServo.setPosition(turnPosDown);
                flipServo.setPosition(flipPosDown);

                pivotServo.setPosition(pivotPosHover);
                pivotServo.setPosition(pivotPosHover);
                gripperServo1.setPosition(gripperPosOpen);

                slideDown = linearSlideMotor.getCurrentPosition() > 100;
                slideUp = false;
                slideMidUp = false;
                slideMidDown = false;
                slideStable = false;
                isTransferring = false;

            }
            // picking up for specimen into human player
            if (gamepad2.dpad_left) {
                turnServo.setPosition(turnPosDown);
                flipServo.setPosition(flipPosDown);

                pivotServo.setPosition(pivotPosHover);
                pivotServo.setPosition(pivotPosHover);
                gripperServo1.setPosition(gripperPosClose);
            }
            // pick up
            if (gamepad2.dpad_down) {
                gripperServo1.setPosition(gripperPosOpen);
                turnServo.setPosition(turnPosDown);
                flipServo.setPosition(flipPosDown);
                sleep(100);
                pivotServo.setPosition(pivotPosDown);
                sleep(200);
//                gripperServo1.setPosition(gripperPosClose);
                slideDown = linearSlideMotor.getCurrentPosition() > 100;
                slideUp = false;
                slideMidUp = false;
                slideMidDown = false;
                slideStable = false;
                isTransferring = false;
            }

            // slide up to basket height
            if (slideUp) {
                if (linearSlideMotor.getCurrentPosition() < slidePosUp) {
                    if (linearSlideMotor.getCurrentPosition() > 3750) {
                        linearSlideMotor.setPower(0.8);
                    } else {
                        linearSlideMotor.setPower(1);
                    }
                }
                if (linearSlideMotor.getCurrentPosition() > slidePosUp) {
                    slideUp = false;
                    linearSlideMotor.setPower(0);
                }
            }

            if (slideStable) {
                if (linearSlideMotor.getCurrentPosition() < slidePosStable) {
                    linearSlideMotor.setPower(1);
                }
                if (linearSlideMotor.getCurrentPosition() > slidePosStable) {
                    slideStable = false;
                    linearSlideMotor.setPower(0);
                }
            }

            //slide up and flip
            if (gamepad1.back) {
                slideUp = true;
                slideMidDown = false;
                slideDown = false;
                slideMidUp = false;
                slideStable = false;
                isTransferring = false;
                sleep(1500);
                flipServo.setPosition(flipPosScore);
                sleep(1000);
                flipServo.setPosition(flipPosDown);
            }

            // slide down
            if (slideDown && !slideMidDown) {
                if (linearSlideMotor.getCurrentPosition() < slidePosDown) {
                    slideDown = false;
                    linearSlideMotor.setPower(0);
                    flipServo.setPosition(flipPosDown);
                } else {
                    linearSlideMotor.setPower(-1);
                }
            }

            if (Math.abs(gripperServo1.getPosition() - 0.05) <= 0.05) {
                currGripperPos = 0;
            } else {
                currGripperPos = 1;
            }
            if (Math.abs(clipServo.getPosition() - 0.05) <= 0.05) {
                currSpecPos = 0;
            } else {
                currSpecPos = 1;
            }


            // flip the bucket
            if (gamepad2.x) {
                flipServo.setPosition(flipPosScore);
                sleep(1000);
                flipServo.setPosition(flipPosDown);
            }

            // Slide all the way up
            if (gamepad2.a) {
                slideUp = true;
                slideDown = false;
                slideMidUp = false;
                slideMidDown = false;
                slideStable = false;
            }

            // Slide all the way down
            if (gamepad2.b) {
                slideDown = linearSlideMotor.getCurrentPosition() > 100;
                slideUp = false;
                slideMidUp = false;
                slideMidDown = false;
                slideStable = false;
                isTransferring = false;
            }
//
//            leftBumper_isPressed = (gamepad2.left_bumper);
//            if (leftBumper_isPressed && !leftBumper_wasPressed) {
//                if (currGripperPos==0) {
//                    gripperServo1.setPosition(gripperPosClose);
//                } else {
//                    gripperServo1.setPosition(gripperPosOpen);
//                }
//            }
//            leftBumper_wasPressed = (gamepad2.left_bumper);

//            rightBumper_isPressed = (gamepad2.right_bumper);
//            if (rightBumper_isPressed && !rightBumper_wasPressed) {
//                if (currSpecPos==0) {
//                    clipServo.setPosition(clipPosOpen);
//                } else {
//                    clipServo.setPosition(clipPosClose);
//                }
//            }
//            rightBumper_wasPressed = (gamepad2.right_bumper);


            if (gamepad2.left_bumper) {
                clipServo.setPosition(clipPosOpen);
                gripperServo1.setPosition(gripperPosOpen);
            }

            if (gamepad2.right_bumper) {
                clipServo.setPosition(clipPosClose);
                gripperServo1.setPosition(gripperPosClose);
            }

            if (gamepad2.right_trigger > 0.3) {
                slideMidUp = true;
                slideMidDown = false;
                slideDown = false;
                slideUp = false;
                isTransferring = false;
                slideStable = false;
                clipServo.setPosition(0);
            }

            if (slideMidUp) {
                if (linearSlideMotor.getCurrentPosition() < slidePosSpecUp) {
                    linearSlideMotor.setPower(0.8);
                } else {
                    linearSlideMotor.setPower(0);
                    slideMidUp = false;
                    if (slideUp) {
                        sleep(1500);
                    }
                }
            }

            if (gamepad2.left_trigger > 0.3) {
                slideMidDown = true;
                slideDown = false;
                slideUp = false;
                slideMidUp = false;
                isTransferring = false;
            }

            if (slideMidDown) {
                if (linearSlideMotor.getCurrentPosition() > slidePosSpecDown) {
                    linearSlideMotor.setPower(-0.8);
                } else {
                    linearSlideMotor.setPower(0);
                    slideMidDown = false;
                    clipServo.setPosition(0.3);
                }
            }

            if (gamepad1.right_bumper) {
                hangSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                hangSlideMotor.setPower(-0.7);
            } else if (gamepad1.left_bumper) {
                if (hangSlideMotor.getCurrentPosition() > -4338) {
                    hangSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                    hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hangSlideMotor.setPower(0.7);
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
                hangSlideMotor2.setPower(0.7 * 0.41);
            } else if (gamepad1.left_trigger >= 0.3F) {
                if (hangSlideMotor.getCurrentPosition() > -4338) {
                    hangSlideMotor2.setDirection(DcMotor.Direction.FORWARD);
                    hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hangSlideMotor2.setPower(-0.7 * 0.41);
                }
            } else {
                if (!isPressedEndOHYE) {
                    hangSlideMotor2.setPower(0);
                }
            }

            if (gamepad1.dpad_up) {
                isPressedEndOHYE = true;
                pivotServo.setPosition(pivotPosTransfer);
            } else {
                if (isPressedEndOHYE) {
                    hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    hangSlideMotor.setPower(-0.7);
                    hangSlideMotor2.setPower(0.7 * 0.41);
                }
            }



            telemetry.addLine("Pivot: " + String.valueOf(pivotServo.getPosition()));
            telemetry.addLine("Turn: " + String.valueOf(turnServo.getPosition()));
            telemetry.addLine("Gripper: " + String.valueOf(gripperServo1.getPosition()));
            telemetry.addLine("Clip: " + String.valueOf(clipServo.getPosition()));
            telemetry.addLine("Flip: " + String.valueOf(flipServo.getPosition()));
            telemetry.addLine("Slide: " + String.valueOf(linearSlideMotor.getCurrentPosition()));
            telemetry.addLine("Hang1: " + String.valueOf(hangSlideMotor.getCurrentPosition()));
            telemetry.addLine("Hang2: " + String.valueOf(hangSlideMotor2.getCurrentPosition()));
            telemetry.addData("slideUp", slideUp);

            telemetry.update();
        }
    }

}
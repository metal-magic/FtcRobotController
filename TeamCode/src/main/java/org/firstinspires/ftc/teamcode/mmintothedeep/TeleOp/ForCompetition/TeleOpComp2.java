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

@TeleOp(name = "!Comp2")
public class TeleOpComp2 extends LinearOpMode {

    public Servo gripperServo1 = null;
    // public Servo gripperServo2 = null;
    public Servo pivotServo = null;
    public Servo turnServo = null;
    public Servo clipServo = null;
    public Servo flipServo = null;
    public DcMotor linearSlideMotor = null;

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

    static final double slidePosDown = 10;
    static final double slidePosSpec = 2400;
    static final double slidePosSamp = 4600;

    static final double pivotPosDown = 0.76;
    static final double pivotPosHover = 0.7;
    static final double pivotPosFloat = 0.5;
    static final double pivotPosTransfer = 0.39;

    static final double turnPosDown = 0.1;
    static final double turnPosTransfer = 0.77;

    static final double flipPosDown = 0;
    static final double flipPosScore = 0.76;

    static final double gripperPosClose = 0.3;
    static final double gripperPosOpen = 0;

    boolean slideMidUp = false;
    boolean slideMidDown = false;

    public void runOpMode() {
        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        // gripperServo2 = hardwareMap.servo.get("gripperServo2");
        pivotServo = hardwareMap.servo.get("pivotServo");

        turnServo = hardwareMap.servo.get("turnServo");

        clipServo = hardwareMap.servo.get("clipServo");

        flipServo = hardwareMap.servo.get("flipServo");

        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        pivotServo.setPosition(0.7083);
        turnServo.setPosition(0.098);
        pivotServo.setPosition(0.7083);
        gripperServo1.setPosition(0);

        waitForStart();


        boolean CutPower = false;
        double motorSpeed;
        while (opModeIsActive()) {

            // Hovering above sample
            if (gamepad1.dpad_right) {
                turnServo.setPosition(0.098);
                pivotServo.setPosition(0.7083);
                gripperServo1.setPosition(0);
            } // Ready to pick up sample
            else if (gamepad1.dpad_left) {
                turnServo.setPosition(0.098);
                gripperServo1.setPosition(0);
                pivotServo.setPosition(0.77);
            }
            if (gamepad1.a) {
                gripperServo1.setPosition(0.3);
            }
            if (gamepad1.b) {
                gripperServo1.setPosition(0);
            }
//            if (gamepad1.x) {
//                pivotServo.setPosition(pivotServo.getPosition() + 0.0001);
//            }
//            if (gamepad1.y) {
//                pivotServo.setPosition(pivotServo.getPosition() - 0.0001);
//            }
//            if (gamepad1.right_trigger >= 0.3F) {
//                gripperServo1.setPosition(gripperServo1.getPosition()+0.005);
//            }
//            if (gamepad1.left_trigger >= 0.3F) {
//                gripperServo1.setPosition(gripperServo1.getPosition()-0.005);
//            }
            if (gamepad1.right_bumper) {
                turnServo.setPosition(turnServo.getPosition()+0.005);
            }
            if (gamepad1.left_bumper) {
                turnServo.setPosition(turnServo.getPosition()-0.005);
            }
            if (gamepad1.dpad_up && clipServo.getPosition() > 0) {
                clipServo.setPosition(clipServo.getPosition()-0.005);
            }
            if (gamepad1.dpad_down && clipServo.getPosition() < 0.3) {
                clipServo.setPosition(clipServo.getPosition()+0.005);
            }
            if (gamepad1.left_stick_button && flipServo.getPosition() < 0.75) {
                flipServo.setPosition(flipServo.getPosition()+0.005);
            }
            if (gamepad1.right_stick_button && flipServo.getPosition() > 0.01) {
                flipServo.setPosition(flipServo.getPosition()-0.005);
            }
            if (linearSlideMotor.getCurrentPosition() < 10000 && gamepad1.right_trigger >= 0.1F) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlideMotor.setPower(0.8);
            } else if (linearSlideMotor.getCurrentPosition() > 50 && gamepad1.left_trigger >= 0.1F) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlideMotor.setPower(-0.8);
            } else {
                if (!slideUp && !slideDown && !slideMidUp && !slideMidDown) {
                    linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    linearSlideMotor.setPower(0);
                }
            }

            // Transfer and slide up
            if (gamepad2.dpad_up) {
                flipServo.setPosition(flipPosDown);
                gripperServo1.setPosition(gripperPosClose);
//                pivotServo.setPosition(pivotPosTransfer);
                //sleep(500);
                turnServo.setPosition(turnPosTransfer);
                sleep(800);
                gripperServo1.setPosition(gripperPosOpen);
                sleep(500);
//                pivotServo.setPosition(pivotPosFloat);
                sleep(500);

                slideUp = true;
                slideDown = false;
                slideMidUp = false;
                slideMidDown = false;
            }
            // align position
            if (gamepad2.dpad_right) {
                turnServo.setPosition(turnPosDown);
                flipServo.setPosition(flipPosDown);

//                pivotServo.setPosition(pivotPosHover);
//                pivotServo.setPosition(pivotPosHover);
                gripperServo1.setPosition(gripperPosOpen);

                slideDown = true;
                slideUp = false;
                slideMidUp = false;
                slideMidDown = false;

            }
            // picking up for specimen into human player
            if (gamepad2.dpad_left) {
                turnServo.setPosition(turnPosDown);
                flipServo.setPosition(flipPosDown);

//                pivotServo.setPosition(pivotPosHover);
//                pivotServo.setPosition(pivotPosHover);
                gripperServo1.setPosition(gripperPosClose);

            }
            // pick up
            if (gamepad2.dpad_down) {
                turnServo.setPosition(turnPosDown);

                flipServo.setPosition(flipPosDown);

//                pivotServo.setPosition(pivotPosDown);
                sleep(200);
                gripperServo1.setPosition(gripperPosClose);

            }

            // slide up and score
            if (slideUp) {
                linearSlideMotor.setPower(1);
                if (linearSlideMotor.getCurrentPosition() > slidePosSamp) {
                    slideUp = false;
                    linearSlideMotor.setPower(0);
                }
            }
            // slide down
            if (slideDown) {
                if (linearSlideMotor.getCurrentPosition() < slidePosDown) {
                    slideDown = false;
                    linearSlideMotor.setPower(slidePosDown);
                    flipServo.setPosition(flipPosDown);
                } else {
                    linearSlideMotor.setPower(-1);
                }
            }

            if (gamepad2.left_bumper) {
                gripperServo1.setPosition(gripperPosClose);
            }
            if (gamepad2.left_bumper) {
                gripperServo1.setPosition(gripperPosOpen);
            }

            // flip the bucket
            if (gamepad2.x) {
                flipServo.setPosition(flipPosScore);
                sleep(1000);
                flipServo.setPosition(flipPosDown);
            }

            // Closes specimen claw
            if (gamepad2.a) {
                clipServo.setPosition(0);
            }

            // Opens specimen clawd
            if (gamepad2.b) {
                clipServo.setPosition(0.3);
            }

            if (gamepad2.left_trigger > 0.3) {
                slideMidUp = true;
                slideMidDown = false;
                slideDown = false;
                slideUp = false;
                clipServo.setPosition(0);
            }

            if (slideMidUp) {
                if (linearSlideMotor.getCurrentPosition() < 3400) {
                    linearSlideMotor.setPower(0.8);
                } else {
                    linearSlideMotor.setPower(0);
                    slideMidUp = false;
                }
            }

            if (gamepad2.right_trigger > 0.3) {
                slideMidDown = true;
                slideDown = false;
                slideUp = false;
                slideMidUp = false;
            }

            if (slideMidDown) {
                if (linearSlideMotor.getCurrentPosition() > 2400) {
                    linearSlideMotor.setPower(-0.8);
                } else {
                    linearSlideMotor.setPower(0);
                    slideDown = false;
                    clipServo.setPosition(0.3);
                }
            }

            telemetry.addLine("Pivot: " + String.valueOf(pivotServo.getPosition()));
            telemetry.addLine("Turn: " + String.valueOf(turnServo.getPosition()));
            telemetry.addLine("Gripper: " + String.valueOf(gripperServo1.getPosition()));
            telemetry.addLine("Clip: " + String.valueOf(clipServo.getPosition()));
            telemetry.addLine("Flip: " + String.valueOf(flipServo.getPosition()));
            telemetry.addLine("Slide: " + String.valueOf(linearSlideMotor.getCurrentPosition()));
            telemetry.addData("slideUp", slideUp);

            telemetry.update();
        }
    }

}
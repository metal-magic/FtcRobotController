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

@TeleOp(name = "!ToggleTest")
public class ToggleTest extends LinearOpMode {

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

    static final double slidePosDown = UtilityValues.SLIDE_POS_DOWN;
    static final double slidePosSpec = UtilityValues.SLIDE_POS_SPEC;
    static final double slidePosSamp = UtilityValues.SLIDE_POS_SAMP;

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

    boolean slideMidUp = false;
    boolean slideMidDown = false;

    boolean A_isPressed = false;
    boolean A_wasPressed = false;
    int currGripperPos = 0;

    boolean B_isPressed = false;
    boolean B_wasPressed = false;
    int currFlipPos = 0;

    boolean X_isPressed = false;
    boolean X_wasPressed = false;
    int currSpecPos = 0;

    boolean Y_isPressed = false;
    boolean Y_wasPressed = false;
    int currTurnPos = 0;

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

            if (Math.abs(gripperServo1.getPosition()-0.05) <= 0.05) {
                currGripperPos = 0;
            } else {
                currGripperPos = 1;
            }

            A_isPressed = (gamepad1.a);
            if (A_isPressed && !A_wasPressed) {
                if (currGripperPos==0) {
                    gripperServo1.setPosition(gripperPosClose);
                } else {
                    gripperServo1.setPosition(gripperPosOpen);
                }
            }
            A_wasPressed = (gamepad1.a);

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
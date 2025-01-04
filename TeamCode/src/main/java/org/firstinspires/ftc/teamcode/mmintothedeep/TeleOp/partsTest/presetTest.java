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

package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.partsTest;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

import java.util.Date;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

/*
  =========================================
  This OpMode was created to test the linear slide motor
  =========================================
 */

@TeleOp(name = "presetTest")
public class presetTest extends OpMode {

    public Servo gripperServo1 = null;
    // public Servo gripperServo2 = null;
    public Servo pivotServo = null;
    public Servo turnServo = null;

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
    @Override
    public void init() {

        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        // gripperServo2 = hardwareMap.servo.get("gripperServo2");
        pivotServo = hardwareMap.servo.get("pivotServo");

        turnServo = hardwareMap.servo.get("turnServo");

        pivotServo.setPosition(0.7083);

    }

    @Override
    public void loop() {

        boolean CutPower = false;
        double motorSpeed;
//        fakeServo.setPosition(1);
//        fakeServo2.setPosition(1);
        if (gamepad2.dpad_right) {
            turnServo.setPosition(0.098);
            pivotServo.setPosition(0.7083);
            gripperServo1.setPosition(0);
        } else if (gamepad2.dpad_left) {
            turnServo.setPosition(0.098);
            gripperServo1.setPosition(0);
//            try {
//                Thread.sleep(300);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
            pivotServo.setPosition(0.77);
        }
        if (gamepad2.back) {
            intake = true;
        } else if (intake) {
            gripperServo1.setPosition(0.3);
            setTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - setTime <= 500) {
                setTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - setTime >= 500) {
                pivotServo.setPosition(0.4072);
                setTime = System.currentTimeMillis();
            }
            while (System.currentTimeMillis() - setTime <= 300) {
                setTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis()-setTime >= 300) {
                turnServo.setPosition(0.76);
                setTime = System.currentTimeMillis();
            }
            while (System.currentTimeMillis() - setTime <= 200) {
                setTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - setTime >= 200) {
                gripperServo1.setPosition(0);
                setTime = System.currentTimeMillis();
            }
            while (System.currentTimeMillis() - setTime <= 200) {
                setTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - setTime >= 200) {
                pivotServo.setPosition(0.5);
            }
            while (System.currentTimeMillis() - setTime <= 100) {
                setTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - setTime >= 100) {
                turnServo.setPosition(0.098);
            }
            intake = false;
        }

        telemetry.update();

    }
}
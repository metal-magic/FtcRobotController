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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.mmcenterstage.HardwareTesting.LeftStrafeTest;
import org.firstinspires.ftc.teamcode.mmcenterstage.other.OldSensorColor2;
import org.firstinspires.ftc.teamcode.mmintothedeep.util.UtilityValues;

import java.util.Date;

/*
  =========================================
  This OpMode was preserved because Om
  wanted his fine controls to be controlled
  with the trigger
  =========================================
 */

@TeleOp(name = "New TeleOp1 Into The Deep")
public class actuatorTest extends OpMode {

    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    public Servo gripperServo1 = null;
    //    public Servo gripperServo2 = null;
    public Servo pivotServo = null;

//    public Servo droneServo = null;

//    public DcMotor linearSlideMotor = null;
    public DcMotor linearActuatorMotor = null;

    public Date previousTime = new Date();

    public float armSpeedCounter = 0;
    // TouchSensor touchSensor = null;
    // OldSensorColor2 board = new OldSensorColor2();

    public static float setPositionCounter = 0;

    @Override
    public void init() {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        gripperServo1 = hardwareMap.servo.get("gripperServo1");
//        gripperServo2 = hardwareMap.servo.get("gripperServo2");
        pivotServo = hardwareMap.servo.get("pivotServo");

//        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        linearActuatorMotor = hardwareMap.dcMotor.get("linearActuatorMotor");

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearActuatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

 //       linearSlideMotor.setDirection(CRServo.Direction.REVERSE);

        ((ServoImplEx) pivotServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
//        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripperServo1.setPosition(0);
        pivotServo.setPosition(0);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // REVERSED
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
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

        if (gamepad1.back && !CutPower) {
            //Button cuts all power except linear slides/actuators
            CutPower = true;
        }
        if (gamepad1.back && CutPower) {
            CutPower = false;
        }

        if (!CutPower) {
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

            if (gamepad2.right_bumper) {
                gripperServo1.setPosition(0.3);
            }
            if (gamepad2.left_bumper) {
                gripperServo1.setPosition(0.1);
            }
            if (gamepad2.dpad_up) {
                gripperServo1.setPosition(0);
            }

            if (gamepad2.y) {
                pivotServo.setPosition(0.5);
            }
            if (gamepad2.a) {
                pivotServo.setPosition(0);
            }
        }
//        Slide limit = 696 mm
//        Slide limit converted to ticks calculation = 537.7*5.7
//        Limit is ROUNDED DOWN
//        3064 max
        double up;
//        if (linearSlideMotor.getCurrentPosition() < 3000 && gamepad2.right_trigger >= 0.1F) {
//            linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
//            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            //linearSlideMotor.setPower(1* UtilityValues.LSSPEED);
//            up = Math.sin(((double) (4000 - linearSlideMotor.getCurrentPosition()) / 4000) * Math.PI / 2);
//            linearSlideMotor.setPower(/*UtilityValues.LSSPEED * */up*gamepad2.right_trigger);
//        } else if (linearSlideMotor.getCurrentPosition() > 100 && gamepad2.left_trigger >= 0.1F) {
//            linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
//            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            ///linearSlideMotor.setPower(-1*UtilityValues.LSSPEED);
//            up = Math.sin(((double) (1000+linearSlideMotor.getCurrentPosition()) /4000)*Math.PI/2);
//            linearSlideMotor.setPower(-1* /*UtilityValues.LSSPEED**/up*gamepad2.left_trigger);
//        } else {
//            if (linearSlideMotor.getCurrentPosition() > 3064) {
//                linearSlideMotor.setPower(-0.3);
//            } else if (linearSlideMotor.getCurrentPosition() < 0) {
//                linearSlideMotor.setPower(0.3);
//            } else {
//                linearSlideMotor.setPower(0);
//            }
//
//        }
        //linear slide limit calculations
        //435/60 = 7.2 revolutions per second
        //1.31 (time it takes to full extend linear actuator at full speed) * 7.2 = 9.36 revolutions per second
        //384.5 * 9.36 = 3595 ticks (little less than actual calculation to be safe)
        if (gamepad1.right_bumper && (linearActuatorMotor.getCurrentPosition()<9100)) {
            linearActuatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearActuatorMotor.setPower(2);
        } else if (gamepad1.left_bumper && (linearActuatorMotor.getCurrentPosition() > 100)){
            linearActuatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearActuatorMotor.setPower(-1);
        } else {
            linearActuatorMotor.setPower(0);
        }

        if (gamepad1.dpad_down) {
            linearActuatorMotor.setPower(-0.5);
        } else if (gamepad1.dpad_up) {
            linearActuatorMotor.setPower(0.5);
        }

        telemetry.addData("Claw Position,", gripperServo1.getPosition());
//        telemetry.addData("Linear Slide Position", linearSlideMotor.getCurrentPosition());
//        telemetry.addData("Linear Slide Speed", linearSlideMotor.getPower());
        telemetry.addData("Linear Actuator Position", linearActuatorMotor.getCurrentPosition());
        telemetry.addData("Linear Actuator Speed", linearActuatorMotor.getPower());
        telemetry.addData("Claw Join Position,", pivotServo.getPosition());

        telemetry.update();

    }
}
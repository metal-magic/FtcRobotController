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


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Date;

/*
  =========================================
  This OpMode was preserved because Om
  wanted his fine controls to be controlled
  with the trigger
  =========================================
 */

@TeleOp(name = "New TeleOp1 Into The Deep")
@Disabled
public class actuatorTest extends OpMode {

    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    public Servo gripperServo1 = null;
    public Servo pivotServo = null;

    public DcMotor linearSlideMotor = null;
    public DcMotor linearActuatorMotor = null;
    public DcMotor linearActuatorMotor2 = null;

    public Date previousTime = new Date();

    public float armSpeedCounter = 0;
    // TouchSensor touchSensor = null;
    // OldSensorColor2 board = new OldSensorColor2();

    public static float setPositionCounter = 0;

    @Override
    public void init() {
        linearActuatorMotor = hardwareMap.dcMotor.get("linearActuatorMotor");
        linearActuatorMotor2 = hardwareMap.dcMotor.get("linearActuatorMotor2");

        linearActuatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linearActuatorMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

 //       hangSlideMotor.setDirection(CRServo.Direction.REVERSE);

        ((ServoImplEx) pivotServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        linearActuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuatorMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper && (linearActuatorMotor.getCurrentPosition()<9100)) {
            linearActuatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearActuatorMotor.setPower(1);
        } else if (gamepad1.left_bumper && (linearActuatorMotor.getCurrentPosition() > 100)){
            linearActuatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearActuatorMotor.setPower(-1);
        } else {
            linearActuatorMotor.setPower(0);
        }

        //auto hang code
        if (gamepad1.dpad_left) {
            while (linearActuatorMotor.getCurrentPosition()<9100) {
                if (gamepad1.dpad_down) {
                    break;
                } else {
                    linearActuatorMotor.setPower(1);
                    linearActuatorMotor2.setPower(1);
                }
            }
        } else if (gamepad1.dpad_right) {
            while (linearActuatorMotor.getCurrentPosition()>100) {
                if (gamepad1.dpad_down) {
                    break;
                } else {
                    linearActuatorMotor.setPower(-1);
                    linearActuatorMotor2.setPower(-1);
                }
            }
        } else {
            linearActuatorMotor2.setPower(0);
            linearActuatorMotor.setPower(0);
        }

        telemetry.addData("Claw Position,", gripperServo1.getPosition());
//        telemetry.addData("Linear Slide Position", hangSlideMotor.getCurrentPosition());
//        telemetry.addData("Linear Slide Speed", hangSlideMotor.getPower());
        telemetry.addData("Linear Actuator Position", linearActuatorMotor.getCurrentPosition());
        telemetry.addData("Linear Actuator Speed", linearActuatorMotor.getPower());
        telemetry.addData("Claw Join Position,", pivotServo.getPosition());

        telemetry.update();

    }
}
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

package org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.competition;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Date;

/*
  =========================================
  This OpMode was created to test the motor directions of all of our chassis's
  =========================================
 */

@TeleOp(name = "testingMotorDirections")
public class motorDirections extends OpMode {

    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;


    public Date previousTime = new Date();

    public static final double motorSpeed = 0.5;

    @Override
    public void init() {

        motorFrontLeft  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        motorFrontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        motorBackLeft   = hardwareMap.get(DcMotor.class, "leftBackDrive");
        motorBackRight  = hardwareMap.get(DcMotor.class, "rightBackDrive");

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);

//        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        boolean CutPower = false;

        if (gamepad1.back && !CutPower) {
            //Button cuts all power except linear slides/actuators
            CutPower = true;
        }
        if (gamepad1.back && CutPower) {
            CutPower = false;
        }

        if (!CutPower) {
            if (gamepad1.a) {
                motorFrontRight.setPower(motorSpeed);
            } else {
                motorFrontRight.setPower(0);
            }
            if (gamepad1.b) {
                motorFrontLeft.setPower(motorSpeed);
            } else {
                motorFrontLeft.setPower(0);
            }
            if (gamepad1.x) {
                motorBackLeft.setPower(motorSpeed);
            } else {
                motorBackLeft.setPower(0);
            }
            if (gamepad1.y) {
                motorBackRight.setPower(motorSpeed);
            } else {
                motorBackRight.setPower(0);
            }
            telemetry.addData("Robot Speed", motorSpeed);
            telemetry.addData("rightFrontDirection", motorFrontRight.getDirection());
            telemetry.addData("leftFrontDirection", motorFrontLeft.getDirection());
            telemetry.addData("leftBackDirection", motorBackLeft.getDirection());
            telemetry.addData("rightBackDirection", motorBackRight.getDirection());

        }

        telemetry.update();

    }
}
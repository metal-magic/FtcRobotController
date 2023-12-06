/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
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

package org.firstinspires.ftc.teamcode.mmcenterstage;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp
public class MMCenterstageTeleOp extends OpMode {

    @Override
    public void init() {
        DcMotor motorFrontLeft = null;
        DcMotor motorFrontRight = null;
        DcMotor motorBackLeft = null;
        DcMotor motorBackRight = null;

        Servo gripperServo1 = null;
        Servo gripperServo2 = null;
        Servo pivotServo = null;

        CRServo armMotor = null;

        // TouchSensor touchSensor = null;


    }

    @Override
    public void loop() {


        // Declare motors
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        Servo gripperServo1 = hardwareMap.servo.get("gripperServo1");
        Servo gripperServo2 = hardwareMap.servo.get("gripperServo2");
        Servo pivotServo = hardwareMap.servo.get("pivotServo");

        // TouchSensor touchSensor = hardwareMap.touchSensor.get("touchSensor");

        CRServo armMotor = hardwareMap.crservo.get("armMotor");


        // Reverse the right side motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setDirection(CRServo.Direction.REVERSE);


        double y = -gamepad1.left_stick_y; // REVERSED
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (abs value) or 1
        // This makes sure that the ratio stays the same
        // but only when at least one is out of range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        if (gamepad2.left_bumper) {
            gripperServo1.setPosition(1);
        } else if (gamepad2.left_trigger == 1.0F) {
            gripperServo1.setPosition(0.2);
        }

        if (gamepad2.right_bumper) {
            gripperServo2.setPosition(0.5);
        } else if (gamepad2.right_trigger == 1.0F) {
            gripperServo2.setPosition(0.65);
        }


        armMotor.setPower(gamepad2.left_stick_y);

        /* if (gamepad2.a) {
            armMotor.setPower(1);
            if (touchSensor.isPressed()) {

                armMotor.setPower(0);
            }
        } */


        if (gamepad2.right_stick_y == 0) {
            pivotServo.setPosition(pivotServo.getPosition());
        } else {
            pivotServo.setPosition(gamepad2.right_stick_y);
        }

    }
}





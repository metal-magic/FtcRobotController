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


import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp
public class MMCenterstageTeleOpTest2 extends OpMode {


    // Declare motors
    DcMotor motorFrontLeft = null;
    DcMotor motorFrontRight = null;
    DcMotor motorBackLeft = null;
    DcMotor motorBackRight = null;

    Servo gripperServo1 = null;
    Servo pivotServo = null;

    CRServo armMotor = null;
    double pivotServoPos = 0.5;
    double pivotServoExpectedPos = pivotServoPos;


    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        pivotServo = hardwareMap.servo.get("pivotServo");
        // TouchSensor touchSensor = hardwareMap.touchSensor.get("touchSensor");

        armMotor = hardwareMap.crservo.get("armMotor");


        // TouchSensor touchSensor = null;

        // Reverse the right side motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setDirection(CRServo.Direction.FORWARD);
        pivotServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // pivotServoPos = pivotServo.getPosition();

        double y = -gamepad1.left_stick_y; // REVERSED
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (abs value) or 1
        // This makes sure that the ratio stays the same
        // but only when at least one is out of range [-1, 1]
        double denominator = max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Get the current position off the buffer
        double pivotServoPos = pivotServo.getPosition();

        telemetry.addLine("Servo Current : " + pivotServoPos);
        telemetry.addLine("Servo PWN Enabled? : " + pivotServo.getController().getPwmStatus());

        motorFrontLeft.setPower(frontLeftPower * 0.75);
        motorBackLeft.setPower(backLeftPower * 0.75);
        motorFrontRight.setPower(frontRightPower * 0.75);
        motorBackRight.setPower(backRightPower * 0.75);

        if (gamepad2.left_bumper) {
            gripperServo1.setPosition(1);
        } else if (gamepad2.left_trigger == 1.0F) {
            gripperServo1.setPosition(0.2);
        }


        armMotor.setPower(gamepad2.left_stick_y * 0.35);

        if (pivotServoPos >=  pivotServoExpectedPos){
            //Do this on when the servo hasn't reached the expected psotion
            pivotServoPos = pivotServoExpectedPos;
            if (gamepad2.dpad_up) {
                pivotServoExpectedPos += 0.1;
                telemetry.addLine("Servo Up :" + pivotServoExpectedPos);
//            if (pivotServoPos > 1) {
//                pivotServoPos = 1;
//            }
            } else if (gamepad2.dpad_down) {
                pivotServoExpectedPos -= 0.1;
                telemetry.addLine("Servo Down :" + pivotServoExpectedPos);
//            if (pivotServoPos < 0) {
//                pivotServoPos = 0;
//            }
            }
            pivotServo.setPosition(pivotServoExpectedPos);
        }else{
            telemetry.addLine("Servo is at : " + pivotServoPos + " and still moving to :" + pivotServoExpectedPos);
        }








        /* if (gamepad2.right_stick_y == 0) {
            pivotServo.setPosition(pivotServo.getPosition());
        }
        else {
            pivotServo.setPosition(gamepad2.right_stick_y);
        } */

        /* if (gamepad2.dpad_up) {
            pivotServo.setPosition(pivotServo.getPosition() + 0.01);
        }
        if (gamepad2.dpad_down) {
            pivotServo.setPosition(pivotServo.getPosition() - 0.01);
        } */






        /* if (gamepad2.dpad_up) {
            pivotServoPos = Math.min(pivotServo.getPosition() + 0.005, 1.0);
        }
        else if (gamepad2.dpad_down) {
            pivotServoPos = Math.max(pivotServo.getPosition() - 0.005, 0);
        }

         */
        // pivotServo.setPosition(pivotServoPos);

        /* if (gamepad2.a) {
            armMotor.setPower(1);
            if (touchSensor.isPressed()) {

                armMotor.setPower(0);
            }
        } */


        /* if (gamepad2.right_stick_y == 0) {
            pivotServo.setPower(pivotServo.getPower());
        } else {
            pivotServo.setPower(gamepad2.right_stick_y);


        }

         */
    }
}





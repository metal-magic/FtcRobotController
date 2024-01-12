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
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Date;

/*
  =========================================
  This OpMode was created because Aditya
  wanted his fine controls to be controlled
  with the D-Pad
  =========================================
 */

// he kinda yelled at me to do it :'( lol - avik

@TeleOp(name = "Aditya's TeleOp 2023-2024 Centerstage")
public class MMCenterstageTeleOpAditya extends OpMode {

    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    public Servo gripperServo1 = null;
    public Servo pivotServo = null;

    public Servo droneServo = null;


    public CRServo armMotor = null;


    public Date previousTime = new Date();

    // TouchSensor touchSensor = null;

    @Override
    public void init() {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        pivotServo = hardwareMap.servo.get("pivotServo");

        droneServo = hardwareMap.servo.get("droneServo");

        // TouchSensor touchSensor = hardwareMap.touchSensor.get("touchSensor");

        armMotor = hardwareMap.crservo.get("armMotor");

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setDirection(CRServo.Direction.REVERSE);

        ((ServoImplEx) pivotServo).setPwmRange(new PwmControl.PwmRange(500, 2500));


    }

    @Override
    public void loop() {
        Date currentTime = new Date();

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

        motorFrontLeft.setPower(frontLeftPower * 0.85);
        motorBackLeft.setPower(backLeftPower * 0.85);
        motorFrontRight.setPower(frontRightPower * 0.85);
        motorBackRight.setPower(backRightPower * 0.85);

        // -------Drivetrain motor speed controls--------- (there is a lot)
        if (gamepad1.dpad_up) {
            // Drive forward at 20 percent normal speed
            motorFrontLeft.setPower(0.30);
            motorBackLeft.setPower(0.30);
            motorFrontRight.setPower(0.30);
            motorBackRight.setPower(0.30);
        }

        if (gamepad1.dpad_down) {
            // Drive backward at 20 percent normal speed
            motorFrontLeft.setPower(-0.20);
            motorBackLeft.setPower(-0.20);
            motorFrontRight.setPower(-0.20);
            motorBackRight.setPower(-0.20);
        }

        if (gamepad1.dpad_right) {
            // Strafe right at 20 percent normal speed
            motorFrontLeft.setPower(0.20);
            motorBackLeft.setPower(-0.20);
            motorFrontRight.setPower(-0.20);
            motorBackRight.setPower(0.20);
        }

        if (gamepad1.dpad_left) {
            // Strafe left at 20 percent normal speed
            motorFrontLeft.setPower(-0.20);
            motorBackLeft.setPower(0.20);
            motorFrontRight.setPower(0.20);
            motorBackRight.setPower(-0.20);
        }

        if (gamepad1.b) {
            // Turn right at 20 percent normal speed
            motorFrontLeft.setPower(0.20);
            motorBackLeft.setPower(0.20);
            motorFrontRight.setPower(-0.20);
            motorBackRight.setPower(-0.20);
        }

        if (gamepad1.x) {
            // Turn left at 20 percent normal speed
            motorFrontLeft.setPower(-0.20);
            motorBackLeft.setPower(-0.20);
            motorFrontRight.setPower(0.20);
            motorBackRight.setPower(0.20);
        }


        // Claw gripper controls
        if (gamepad2.right_bumper) {
            gripperServo1.setPosition(0.6);
        }
        if (gamepad2.left_bumper) {
            gripperServo1.setPosition(0.2);
        }

        // Arm motor speed controls
        double armMotorSpeed;

        if (gamepad2.right_trigger >= 0.3F) {
            // Fine controls
            armMotorSpeed = 0.20;
        } else {
            // Reg speed
            armMotorSpeed = 0.35;
        }

        armMotor.setPower(gamepad2.right_stick_y * armMotorSpeed);

        // Pivot servo control code
        if (currentTime.getTime() - previousTime.getTime() > 100) {
            double pivotIncrement;

            if (gamepad2.left_trigger >= 0.3F) {
                pivotIncrement = 0.01;
            } else {
                pivotIncrement = 0.05;
            }
            if (gamepad2.dpad_up) {
                telemetry.addLine("Servo Will go Up");
                pivotServo.setPosition(pivotServo.getPosition() - pivotIncrement);
            }


            if (gamepad2.dpad_down) {
                telemetry.addLine("Servo Will go down");
                pivotServo.setPosition(pivotServo.getPosition() + pivotIncrement);
            }

            previousTime = currentTime;

        }

        if (gamepad2.a) {
            pivotServo.setPosition(0.4);
        }
        if (gamepad2.b) {
            pivotServo.setPosition(0);
        }

        if (gamepad2.y) {
            droneServo.setPosition(0);
        }
        telemetry.addLine("pivotServo position:" + pivotServo.getPosition());

    }
}






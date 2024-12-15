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


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mmcenterstage.HardwareTesting.LeftStrafeTest;
import org.firstinspires.ftc.teamcode.mmcenterstage.other.OldSensorColor2;
import org.firstinspires.ftc.teamcode.mmintothedeep.util.UtilityValues;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Date;
import java.util.Objects;
import java.util.Timer;


/*
  =========================================
  This OpMode was preserved because Om
  wanted his fine controls to be controlled
  with the trigger
  =========================================
 */

@TeleOp(name = "rightAutoTest")
@Disabled
public class rightAutoTest extends OpMode {

    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;


//    public Servo droneServo = null;

    //public DcMotor linearActuatorMotor = null;

    public Date previousTime = new Date();


    public static float setPositionCounter = 0;

    static final double MOTOR_TICK_COUNTS = UtilityValues.motorTicks; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER_INCHES = UtilityValues.wheelDiameter / 25.4; // in Inches
    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches

    public double DEGREES_MOTOR_MOVES_IN_1_REV = 56;

    static final double SPEED = UtilityValues.SPEED; // Motor Power setting


    @Override
    public void init() {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");


        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //linearActuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        telemetry.addData("Cutpower", CutPower);
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
            telemetry.addData("MotorFrontLeft", frontLeftPower);
            motorBackLeft.setPower(backLeftPower * motorSpeed);
            telemetry.addData("MotorBackLeft", frontLeftPower);
            motorFrontRight.setPower(frontRightPower * motorSpeed);
            telemetry.addData("MotorFrontRight", frontLeftPower);
            motorBackRight.setPower(backRightPower * motorSpeed);
            telemetry.addData("MotorBackRight", frontLeftPower);
        }

//        if (gamepad1.y) {
//            DEGREES_MOTOR_MOVES_IN_1_REV += 0.1;
//        }
//
//        if (gamepad1.x) {
//            DEGREES_MOTOR_MOVES_IN_1_REV -= 0.1;
//        }
//
//        if (gamepad1.a) {
//            rotate(360);
//        }




        telemetry.addData("Rotation Coefficient: ", DEGREES_MOTOR_MOVES_IN_1_REV);

    }

    public void rotate (double degrees) {
        double robotSpeed = SPEED;
        // Assume positive degrees means moving towards the right
        double movementOfWheelsInRevs = Math.abs(degrees / DEGREES_MOTOR_MOVES_IN_1_REV);

        if (degrees >= 0) {
            drive(robotSpeed,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs);
        } else {
            // Moving negative means rotating left
            drive(robotSpeed,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs);
        }
    }

    private void strafe(double strafeInches) {
        // We assume that strafing right means positive
        double strafeRevs = Math.abs(strafeInches / CIRCUMFERENCE_INCHES);
        if (strafeInches >= 0) {
            telemetry.addData("Strafing towards right by ", "%.3f inches", strafeInches);

            drive(SPEED,
                    1 * strafeRevs,
                    -1 * strafeRevs,
                    -1 * strafeRevs,
                    1 * strafeRevs);
        } else {
            telemetry.addData("Strafing towards Left by ", "%.3f inches", Math.abs(strafeInches));

            drive(SPEED,
                    -1 * strafeRevs,
                    1 * strafeRevs,
                    1 * strafeRevs,
                    -1 * strafeRevs);
        }
    }

    /*
    =====================================================
    MOVE IN STRAIGHT LINE FUNCTION
    to call:
        moveStraightLine(# of inches);
        positive # of inches -> forward
    =====================================================
    */
    private void moveStraightLine(double movementInInches) {
        double moveInRevs = movementInInches / CIRCUMFERENCE_INCHES;
        telemetry.addData("Moving ", "%.3f inches", movementInInches);
        telemetry.update();
        drive(SPEED, moveInRevs, moveInRevs, moveInRevs, moveInRevs);
    }

    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs, double rightBackRevs) {

        int LFdrivetarget = (int) (leftFrontRevs * MOTOR_TICK_COUNTS) + motorFrontLeft.getCurrentPosition();
        int LBdrivetarget = (int) (leftBackRevs * MOTOR_TICK_COUNTS) + motorBackLeft.getCurrentPosition();
        int RFdrivetarget = (int) (rightFrontRevs * MOTOR_TICK_COUNTS) + motorFrontRight.getCurrentPosition();
        int RBdrivetarget = (int) (rightBackRevs * MOTOR_TICK_COUNTS) +  motorBackRight.getCurrentPosition();

        motorFrontLeft.setTargetPosition(LFdrivetarget);
        motorBackLeft.setTargetPosition(LBdrivetarget);
        motorFrontRight.setTargetPosition(RFdrivetarget);
        motorBackRight.setTargetPosition(RBdrivetarget);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorFrontLeft.setPower(speed);
        motorBackLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);

        while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() || motorFrontRight.isBusy() || motorBackRight.isBusy()) {
//            telemetry.addLine("Current Position of the Motors")
//                    .addData("Left Front  ", "%d", leftFrontDrive.getCurrentPosition())
//                    .addData("Left Back ", "%d", leftBackDrive.getCurrentPosition())
//                    .addData("Right Front ", "%d", rightFrontDrive.getCurrentPosition())
//                    .addData("Right Back ", "%df", rightBackDrive.getCurrentPosition());
//
//            telemetry.addLine("Target Positions of the Motors")
//                    .addData("Left Front  ", "%d", LFdrivetarget)
//                    .addData("Left Back ", "%d", LBdrivetarget)
//                    .addData("Right Front ", "%d", RFdrivetarget)
//                    .addData("Right Back ", "%df", RBdrivetarget);

            //telemetry.update();
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);


    }

}
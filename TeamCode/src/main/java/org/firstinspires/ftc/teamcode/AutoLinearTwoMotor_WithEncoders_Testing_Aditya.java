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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * I used this youtube tutorial link from FTC Canada:    https://www.youtube.com/watch?v=c__eqpm2vc0
 */

@Autonomous(name="Robot: Auto Drive By Encoder2", group="Robot")
public class AutoLinearTwoMotor_WithEncoders_Testing_Aditya extends LinearOpMode {


    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftBackDrive;

    private int leftFrontDrivePos;
    private int rightFrontDrivePos;
    private int rightBackDrivePos;
    private int leftBackDrivePos;


    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");


        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrivePos = 0;
        rightBackDrivePos = 0;
        leftBackDrivePos = 0;
        rightFrontDrivePos = 0;

        waitForStart();

        drive(1000, 1000, 1000,1000, .25);
        drive(-1000, 1000, -1000,1000, .25);
    }

    private void drive(int leftFrontTarget, int rightBackTarget, int leftBackTarget, int rightFrontTarget, double speed) {
        leftFrontDrivePos += leftFrontTarget;
        rightBackDrivePos += rightBackTarget;
        leftBackDrivePos += leftBackTarget;
        rightFrontDrivePos += rightFrontTarget;

        leftFrontDrive.setTargetPosition(leftFrontDrivePos);
        rightBackDrive.setTargetPosition(rightBackDrivePos);
        leftBackDrive.setTargetPosition(leftBackDrivePos);
        rightFrontDrive.setTargetPosition(rightFrontDrivePos);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        leftFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        while(opModeIsActive() && leftFrontDrive.isBusy() && rightBackDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy()) {
            idle();
        }

    }


}

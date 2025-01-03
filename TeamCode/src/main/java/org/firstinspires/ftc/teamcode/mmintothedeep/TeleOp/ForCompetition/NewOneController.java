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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

/*
  =========================================
  This OpMode was preserved because Om
  wanted his fine controls to be controlled
  with the trigger
  =========================================
 */

@TeleOp(name = "NEW ONE CONTROLLER TeleOp")
public class NewOneController extends OpMode {

    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public Servo gripperServo1 = null;
    // public Servo gripperServo2 = null;
    public Servo pivotServo = null;
    public Servo turnServo = null;
    // public Servo fakeServo = null;
    // public Servo fakeServo2 = null;

    // public Servo droneServo = null;

    public DcMotor linearSlideMotor = null;
    public DcMotor hangSlideMotor = null;
    public DcMotor hangSlideMotor2 = null;

    boolean isPressed = false;
    boolean isPressed2 = false;

    boolean isPresetPressed = false;
    boolean wasPresetPressed = false;
    boolean swPreset = false;

    boolean isPressedEndOHYE = false;
    // public DcMotor linearActuatorMotor = null;

    //public Date previousTime = new Date();

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

    boolean moveSlideUp = false;
    boolean moveSlideDown = false;
    boolean intake = false;
    long setTime;
    @Override
    public void init() {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        // gripperServo2 = hardwareMap.servo.get("gripperServo2");
        pivotServo = hardwareMap.servo.get("pivotServo");
        // fakeServo = hardwareMap.servo.get("fakeServo");
        // fakeServo2 = hardwareMap.servo.get("fakeServo2");

        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        hangSlideMotor = hardwareMap.dcMotor.get("hangSlideMotor1");
        hangSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangSlideMotor2 = hardwareMap.dcMotor.get("hangSlideMotor2");
        hangSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // linearActuatorMotor = hardwareMap.dcMotor.get("linearActuatorMotor");

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // linearActuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // linearActuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        gripperServo1.setPosition(0);
//        pivotServo.setPosition(0.673);

        hangSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangSlideMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up) {
            moveSlideUp = true;
            moveSlideDown = false;
        }
        if (gamepad2.dpad_right) {
            moveSlideDown = true;
            moveSlideUp = false;
        }
        if (gamepad2.dpad_left) {
            moveSlideDown = true;
        }
        if (moveSlideDown) {
            moveSlideUp = false;
        }
        if (moveSlideUp) {
            moveSlideDown = false;
        }

        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
         * GAMEPAD 2 IS MAIN AND GAMEPAD 1 IS HANGING
         * 
         */
        double y = -gamepad2.left_stick_y + gamepad1.left_stick_y/2; // REVERSED -gamepad1.left_stick_y.gamestick so
        // gamepad1 can also do movement for hanging
        // making sure it doesnt go over 1 or -1
        if (y < -1) {
            y = -1;
        } else if (y > 1) {
            y = 1;
        }
        double x = gamepad2.left_stick_x - gamepad1.left_stick_x/2; // gamepad1 can also do movement for hanging
        // making sure it doesnt go over 1 or -1
        if (x > 1) {
            x = 1;
        } else if (x < -1) {
            x = -1;
        }
        double rx = gamepad2.right_stick_x + gamepad1.right_stick_x/2; // gamepad1 can also do movement for hanging
        // making sure it doesnt go over 1 or -1
        if (rx > 1) {
            rx = 1;
        } else if (rx < -1) {
            rx = -1;
        }

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
        // fakeServo.setPosition(1);
        // fakeServo2.setPosition(1);


            telemetry.addData("slide", linearSlideMotor.getCurrentPosition());

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

            isPresetPressed = gamepad1.b;

            if (isPresetPressed && !wasPresetPressed) {
                swPreset = !swPreset;
            }

            wasPresetPressed = gamepad1.b;

            if (gamepad2.left_bumper) {
                gripperServo1.setPosition(0);
            } else if (gamepad2.right_bumper) {
                gripperServo1.setPosition(0.3);
            }

            if (gamepad2.y) {
                pivotServo.setPosition(pivotServo.getPosition() + 0.009);
            } else if (gamepad2.a) {
                if (pivotServo.getPosition() < 0.09) {
                    pivotServo.setPosition(0.10);
                } else {
                    pivotServo.setPosition(pivotServo.getPosition() - 0.009);
                }
            } else if (gamepad2.dpad_up) {
                pivotServo.setPosition(0.4072);
            } else if (gamepad2.dpad_down) {
                pivotServo.setPosition(0.7583);
            }
            if (gamepad2.dpad_right) {
                turnServo.setPosition(0.098);
                pivotServo.setPosition(0.7083);
                gripperServo1.setPosition(0);
            } else if (gamepad2.dpad_left) {
                setTime = System.currentTimeMillis();
                turnServo.setPosition(0.098);
                gripperServo1.setPosition(0);
                if (System.currentTimeMillis() - setTime > 300) {
                    pivotServo.setPosition(0.7583);
                }
            }

        if (linearSlideMotor.getCurrentPosition() < 10000 && gamepad2.right_trigger >= 0.1F) {
            linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(1);
        } else if (linearSlideMotor.getCurrentPosition() > 50 && gamepad2.left_trigger >= 0.1F) {
            linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(-1);
        } else if (moveSlideUp) {
            if (linearSlideMotor.getCurrentPosition() < 4000) {
                linearSlideMotor.setPower(0.7);
            } else {
                linearSlideMotor.setPower(0);
                moveSlideUp = false;
            }
        } else if (moveSlideDown) {
            if (linearSlideMotor.getCurrentPosition() > 10) {
                linearSlideMotor.setPower(-0.7);
            } else {
                pivotServo.setPosition(0.7583);
                linearSlideMotor.setPower(0);
                moveSlideDown = false;
            }
        } else {
            linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(0);
        }

        if (gamepad1.dpad_left) {
            isPressed = false;
        }

        if (gamepad1.right_bumper) {
            hangSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            hangSlideMotor.setPower(-0.7);
        } else if (gamepad1.left_bumper) {
            if (hangSlideMotor.getCurrentPosition() > -4338) {
                hangSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hangSlideMotor.setPower(0.7);
            }
        } else {
            if (!isPressedEndOHYE) {
                hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hangSlideMotor.setPower(0);
            }
        }

        if (gamepad1.right_trigger >= 0.3F) {
            hangSlideMotor2.setDirection(DcMotor.Direction.FORWARD);
            hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangSlideMotor2.setPower(0.7 * 0.41);
        } else if (gamepad1.left_trigger >= 0.3F) {
            if (hangSlideMotor.getCurrentPosition() > -4338) {
                hangSlideMotor2.setDirection(DcMotor.Direction.FORWARD);
                hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hangSlideMotor2.setPower(-0.7 * 0.41);
            }
        } else {
            if (!isPressedEndOHYE) {
                hangSlideMotor2.setPower(0);
            }
        }

        if (gamepad1.dpad_up) {
            isPressedEndOHYE = true;
            pivotServo.setPosition(0.31);
        } else {
            if (isPressedEndOHYE) {
                hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hangSlideMotor.setPower(-0.7);
                hangSlideMotor2.setPower(0.7 * 0.41);
            }
        }

        if (gamepad2.back) {
            intake = true;
        } else if (intake) {
            gripperServo1.setPosition(0.3);
            setTime = System.currentTimeMillis();
            if (System.currentTimeMillis() - setTime > 500) {
                pivotServo.setPosition(0.4072);
                setTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis()-setTime > 300) {
                turnServo.setPosition(0.76);
                setTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - setTime > 200) {
                gripperServo1.setPosition(0);
                setTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - setTime > 100) {
                turnServo.setPosition(0.098);
            }
            intake = false;
        }

        telemetry.addData("hang Slide position, ", hangSlideMotor.getCurrentPosition());
        telemetry.addData("hang Slide2 position", hangSlideMotor2.getCurrentPosition());

        telemetry.addData("Claw Position,", gripperServo1.getPosition());
        // telemetry.addData("Linear Slide Position",
        // hangSlideMotor.getCurrentPosition());
        // telemetry.addData("Linear Slide Speed", hangSlideMotor.getPower());
        // telemetry.addData("Linear Actuator Position",
        // linearActuatorMotor.getCurrentPosition());
        // telemetry.addData("Linear Actuator Speed", linearActuatorMotor.getPower());
        telemetry.addData("Claw Join Position,", pivotServo.getPosition());
        telemetry.addData("slide", linearSlideMotor.getCurrentPosition());
        telemetry.addData("Hanging slide 1", hangSlideMotor.getCurrentPosition());
        telemetry.addData("Hanging slide 2", hangSlideMotor2.getCurrentPosition());

        telemetry.update();

    }

    public void rotate(double degrees) {
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
     * =====================================================
     * MOVE IN STRAIGHT LINE FUNCTION
     * to call:
     * moveStraightLine(# of inches);
     * positive # of inches -> forward
     * =====================================================
     */
    private void moveStraightLine(double movementInInches) {
        double moveInRevs = movementInInches / CIRCUMFERENCE_INCHES;
        telemetry.addData("Moving ", "%.3f inches", movementInInches);
        telemetry.update();
        drive(SPEED, moveInRevs, moveInRevs, moveInRevs, moveInRevs);
    }

    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs,
            double rightBackRevs) {

        int LFdrivetarget = (int) (leftFrontRevs * MOTOR_TICK_COUNTS) + leftFrontDrive.getCurrentPosition();
        int LBdrivetarget = (int) (leftBackRevs * MOTOR_TICK_COUNTS) + leftBackDrive.getCurrentPosition();
        int RFdrivetarget = (int) (rightFrontRevs * MOTOR_TICK_COUNTS) + rightFrontDrive.getCurrentPosition();
        int RBdrivetarget = (int) (rightBackRevs * MOTOR_TICK_COUNTS) + rightBackDrive.getCurrentPosition();

        leftFrontDrive.setTargetPosition(LFdrivetarget);
        leftBackDrive.setTargetPosition(LBdrivetarget);
        rightFrontDrive.setTargetPosition(RFdrivetarget);
        rightBackDrive.setTargetPosition(RBdrivetarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy()
                || rightBackDrive.isBusy()) {
            // telemetry.addLine("Current Position of the Motors")
            // .addData("Left Front ", "%d", leftFrontDrive.getCurrentPosition())
            // .addData("Left Back ", "%d", leftBackDrive.getCurrentPosition())
            // .addData("Right Front ", "%d", rightFrontDrive.getCurrentPosition())
            // .addData("Right Back ", "%df", rightBackDrive.getCurrentPosition());
            //
            // telemetry.addLine("Target Positions of the Motors")
            // .addData("Left Front ", "%d", LFdrivetarget)
            // .addData("Left Back ", "%d", LBdrivetarget)
            // .addData("Right Front ", "%d", RFdrivetarget)
            // .addData("Right Back ", "%df", RBdrivetarget);

            // telemetry.update();
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

}
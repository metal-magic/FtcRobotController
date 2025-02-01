package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.partsTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorRunToPosition extends OpMode {

    public DcMotor testMotor = null;
    int newTarget;

    @Override
    public void init() {
        testMotor = hardwareMap.dcMotor.get("testMotor");

        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {

            runToPosition(testMotor, 1500, 0.5);

        }

    }

    public void runToPosition(DcMotor motor, int ticks, double power) {
        newTarget = ticks;
        motor.setTargetPosition(newTarget);
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}

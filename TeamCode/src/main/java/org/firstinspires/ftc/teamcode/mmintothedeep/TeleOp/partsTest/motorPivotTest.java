package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.partsTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class motorPivotTest extends OpMode {

    DcMotor testMotor;

    @Override
    public void init() {
        testMotor = hardwareMap.dcMotor.get("testMotor");

        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad1.dpad_up) {
            testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad1.dpad_down) {
            testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        testMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        telemetry.addData("Motor Position", testMotor.getCurrentPosition());


    }

}

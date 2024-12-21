package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//Use right and left trigger on gamepad 1 to test if the servo works
public class servoTest extends OpMode {
    public Servo testServo = null;

    @Override
    public void init() {
        testServo = hardwareMap.servo.get("testServo");
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            testServo.setPosition(1);
        } else if (gamepad1.left_bumper) {
            testServo.setPosition(0);
        }
    }
}

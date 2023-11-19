package org.firstinspires.ftc.teamcode.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoManualTest extends OpMode {

    Servo servo1 = null;
    Servo servo2 = null;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            servo1.setPosition(1);
        }
        else if (gamepad1.left_trigger == 1.0F) {
            servo1.setPosition(0.2);
        }

        if (gamepad1.right_bumper) {
            servo2.setPosition(0.5);
        }
        else if (gamepad1.right_trigger == 1.0F) {
            servo2.setPosition(0.65);
        }

    }
}

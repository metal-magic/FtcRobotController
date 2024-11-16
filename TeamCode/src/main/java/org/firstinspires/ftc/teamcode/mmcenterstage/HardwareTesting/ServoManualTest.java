package org.firstinspires.ftc.teamcode.mmcenterstage.HardwareTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name="Test All Servos - ONLY IF NEEDED", group="Testing")
@Disabled
public class ServoManualTest extends OpMode {

    CRServo pivotServo = null;
    Servo gripperServo1 = null;
    Servo gripperServo2 = null;

    @Override
    public void init() {
        pivotServo = hardwareMap.get(CRServo.class, "pivotServo");
        gripperServo1 = hardwareMap.get(Servo.class, "gripperServo1");
        gripperServo2 = hardwareMap.get(Servo.class, "gripperServo2");
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            gripperServo1.setPosition(1);
        }
        else if (gamepad1.left_trigger == 1.0F) {
            gripperServo1.setPosition(0.2);
        }

        if (gamepad1.right_bumper) {
            gripperServo2.setPosition(0.5);
        }
        else if (gamepad1.right_trigger == 1.0F) {
            gripperServo2.setPosition(0.65);

            gripperServo2.setPosition(0.65);
        }

        if(gamepad1.left_stick_y < 0) {
            pivotServo.setPower(gamepad1.left_stick_y);
        }
        else if (gamepad1.left_stick_y > 0) {
            pivotServo.setPower(gamepad1.left_stick_y);
        }
        else {
            pivotServo.setPower(0);
        }




    }
}

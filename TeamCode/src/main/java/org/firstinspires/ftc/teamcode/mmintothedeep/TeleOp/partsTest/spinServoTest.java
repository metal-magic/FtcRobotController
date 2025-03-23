package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.partsTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * =======================================================================
 *
 * OP MODE TO TEST THE SERVO
 * Please read:
 * 1. In configuration, name the servo as 'testServo'
 * 2. Find the TeleOp that is called: 'Servo Test'
 * GAMEPAD 1:
 * 3. left trigger -> servo position 0
 * 4. right trigger -> servo position 1
 * 5. Y to increase servo position slightly and A to decrease
 * 6. Servo position is on telemetry data
 *
 * =======================================================================
 */

@TeleOp(group = "parts", name = "Servo Test")
@Disabled
public class spinServoTest extends OpMode {
    public Servo testServo = null;

    @Override
    public void init() {
        testServo = hardwareMap.servo.get("turnServo");
//        testServo.setPosition(0);
    }

    @Override
    public void loop() {


        // servo controls
        if (gamepad1.right_bumper) {
            testServo.setPosition(1);
        } else if (gamepad1.left_bumper) {
            testServo.setPosition(0);
        } else if (gamepad1.y) {
            testServo.setPosition(testServo.getPosition()+0.001);
        } else if (gamepad1.a) {
            testServo.setPosition(testServo.getPosition()-0.001);
        }

        // add to telemetry
        telemetry.addData("Servo position", testServo.getPosition());
        telemetry.update();
    }
}

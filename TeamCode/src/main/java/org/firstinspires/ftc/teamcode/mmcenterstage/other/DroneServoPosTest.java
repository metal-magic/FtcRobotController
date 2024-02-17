package org.firstinspires.ftc.teamcode.mmcenterstage.other;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Date;

@TeleOp
public class DroneServoPosTest extends OpMode {

    Servo droneServo;

    Date previousTime = new Date();

    @Override
   public void init() {

        droneServo = hardwareMap.servo.get("droneServo");

        ((ServoImplEx) droneServo).setPwmRange(new PwmControl.PwmRange(500, 2500));

    }

    @Override
    public void loop() {
        Date currentTime = new Date();

        if (currentTime.getTime() - previousTime.getTime() > 100) {
            double pivotIncrement;

            if (gamepad2.left_trigger >= 0.3F) {
                pivotIncrement = 0.01;
            } else {
                pivotIncrement = 0.05;
            }
            if (gamepad1.dpad_up) {
                telemetry.addLine("Servo Will go Up");
                droneServo.setPosition(droneServo.getPosition() - pivotIncrement);
            }


            if (gamepad1.dpad_down) {
                telemetry.addLine("Servo Will go down");
                droneServo.setPosition(droneServo.getPosition() + pivotIncrement);
            }

            previousTime = currentTime;
            telemetry.addLine("dronServo position:" + droneServo.getPosition());
        }

    }
}

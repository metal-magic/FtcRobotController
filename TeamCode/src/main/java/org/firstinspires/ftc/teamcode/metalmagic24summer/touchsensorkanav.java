package org.firstinspires.ftc.teamcode.metalmagic24summer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Touch Sensor Example", group="Test")
class DistanceSensor extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize the touch sensor
        TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Check if the touch sensor is pressed
            if (touchSensor.isPressed()) {
                telemetry.addData("Touch Sensor", "Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Not Pressed");
            }

            // Update the telemetry data
            telemetry.update();
        }
    }
}


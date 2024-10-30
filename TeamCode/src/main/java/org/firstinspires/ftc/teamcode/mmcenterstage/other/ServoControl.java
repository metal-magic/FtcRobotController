package org.firstinspires.ftc.teamcode.mmcenterstage.other;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "ServoControl", group = "Linear Opmode")
@Disabled
public class ServoControl extends LinearOpMode {


    private static final double SERVO_POSITION_60_DEGREES = 60.0 / 180.0; // Convert degrees to servo position (0 to 1 range)
    private static final double SERVO_INITIAL_POSITION = 0.0;


    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        Servo servo = hardwareMap.get(Servo.class, "servo");


        // Set the initial position of the servo
        servo.setPosition(SERVO_INITIAL_POSITION);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Check if the left bumper is pressed
            if (gamepad1.left_bumper) {
                servo.setPosition(SERVO_POSITION_60_DEGREES);
            }


            // Check if the left trigger is pressed
            if (gamepad1.left_trigger > 0.5) {
                servo.setPosition(SERVO_INITIAL_POSITION);
            }


            // Send telemetry data to the driver station
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}

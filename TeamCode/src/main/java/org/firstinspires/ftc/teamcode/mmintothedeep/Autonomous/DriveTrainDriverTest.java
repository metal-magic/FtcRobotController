package org.firstinspires.ftc.teamcode.mmintothedeep.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mmintothedeep.util.DriveTrain.DriveTrainDriver;

import java.util.Date;

public class DriveTrainDriverTest extends LinearOpMode {
    Date currentTime = new Date();
    static final double SPEED = 0.5; // Motor Power setting

    @Override
    public void runOpMode() {
        waitForStart();
        // Creates an object for DriveTrainDriver class
        DriveTrainDriver testChassis = new DriveTrainDriver(hardwareMap, true, true, true, true);

        // Moves the robot forward
        testChassis.drive(SPEED, 1, 1, 1, 1);

        // Prevents multiple commands from occurring too quickly
        sleep(100);

        // Moves the robot backward
        testChassis.drive(SPEED, -1, -1, -1, -1);

        sleep(100);

        // Moves the robot to the right
        testChassis.drive(SPEED, 1, -1, -1, 1);

        sleep(100);

        // Moves the robot to the left
        testChassis.drive(SPEED, -1, 1, 1, -1);

        // Termination
        if (currentTime.getTime() > 20000) {
            testChassis.terminate();
        }

    }
}

package org.firstinspires.ftc.teamcode.mmintothedeep.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mmintothedeep.util.DriveTrain.DriveTrainDriver;

import java.util.Date;

@Autonomous
@Disabled
public class DriveTrainDriverTest extends LinearOpMode {
    Date currentTime = new Date();
    static final double SPEED = 0.5; // Motor Power setting

    @Override
    public void runOpMode() {
        waitForStart();
        // Creates an object for DriveTrainDriver class
        DriveTrainDriver testChassis = new DriveTrainDriver(hardwareMap, true, true, true, true);

        while (!isStopRequested() && opModeIsActive()) {
            // Moves the robot forward
            testChassis.moveStraightLine(3, 0.5);

            // Prevents multiple commands from occurring too quickly
            sleep(1000);

            // Moves the robot backward
            testChassis.moveStraightLine(-3, 0.5);

            sleep(1000);

            // Moves the robot to the right
            testChassis.drive(SPEED, 1, -1, -1, 1);

            sleep(1000);

            // Moves the robot to the left
            testChassis.drive(SPEED, -1, 1, 1, -1);
            // Termination
            if (currentTime.getTime() > 20000000) {
                testChassis.terminate();
                break;
            }

        }



    }
}

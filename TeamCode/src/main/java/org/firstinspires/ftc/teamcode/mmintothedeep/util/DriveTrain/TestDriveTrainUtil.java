package org.firstinspires.ftc.teamcode.mmintothedeep.util.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.mmintothedeep.util.UtilityValues;

@Autonomous(name="Test: Drive Train Util Class")
public class TestDriveTrainUtil extends LinearOpMode {

    //public static final double SPEED = UtilityValues.SPEED;
    //FOR SPEED, UtilityValues class SPEED is already in DriveTrainUtil

    HardwareMap hardwareMap = null;

    // Created a Drive object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    DriveTrainUtil robot = new DriveTrainUtil(this);

    @Override
    public void runOpMode() {

        robot.init();
        waitForStart();

        //move forward and back 5 in
        robot.moveStraightLine(5);
        sleep(250);
        robot.moveStraightLine(-5);
        sleep(250);

        //strafe left and right 5 in
        robot.strafe(5);
        sleep(250);
        robot.strafe(5);
        sleep(250);

        //rotate left and right 90 degrees
        robot.rotate(-90);
        sleep(250);
        robot.rotate(90);
        sleep(250);

//        while (!isStopRequested() && opModeIsActive()) {
//
//
//
//        }

    }

}

package cc.metalmagic.robot.components.tests;

import static cc.metalmagic.robot.components.tests.ComponentTest.Status.RUNNING;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import cc.metalmagic.robot.RobotConfigurationConstants;
import cc.metalmagic.robot.components.tests.Exceptions.TestFailedException;
import cc.metalmagic.robot.components.tests.Exceptions.TestNotInitializedException;


@Autonomous(name="Robot Hardware Test Runner", group="Robot")
public class HardwareTestRunner  extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Test Motors
        TestDCMotorRunUsingEncoder frontLeft = new TestDCMotorRunUsingEncoder(
                hardwareMap.get(DcMotor.class, RobotConfigurationConstants.FRONT_LEFT_MOTOR),
                0, telemetry);
        TestDCMotorRunUsingEncoder frontRight = new TestDCMotorRunUsingEncoder(
                hardwareMap.get(DcMotor.class, RobotConfigurationConstants.FRONT_RIGHT_MOTOR),
        1, telemetry);
        TestDCMotorRunUsingEncoder backLeft = new TestDCMotorRunUsingEncoder(
                hardwareMap.get(DcMotor.class, RobotConfigurationConstants.BACK_LEFT_MOTOR),
        2, telemetry);
        TestDCMotorRunUsingEncoder backRight = new TestDCMotorRunUsingEncoder(
                hardwareMap.get(DcMotor.class, RobotConfigurationConstants.BACK_RIGHT_MOTOR),
                        3, telemetry);
        ComponentTest[] tests = new ComponentTest[]{frontLeft, frontRight, backLeft, backRight};
        // ComponentTest[] tests = new ComponentTest[]{frontLeft};
        waitForStart();
        runTests(tests);
        sleep(20000);
    }

    void runTests(ComponentTest[] tests) {
        telemetry.setAutoClear(false);
        for (ComponentTest test : tests) {
            try {
                test.init(0, telemetry);
                test.runTestsInternal();
                while (test.getStatus() == RUNNING) {

                }
            } catch (TestFailedException e) {
                telemetry.addLine(e.getMessage());
                telemetry.update();
            } catch (TestNotInitializedException e) {
                telemetry.addLine(e.getMessage());
                telemetry.update();
            }
        }
    }
}

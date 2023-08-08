package cc.metalmagic.robot.components.tests;

import static cc.metalmagic.robot.components.tests.ComponentTest.Status.FAILED;
import static cc.metalmagic.robot.components.tests.ComponentTest.Status.RUNNING;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name="Robot Hardware Test Runner", group="Robot")
public class HardwareTestRunner  extends LinearOpMode {
    @Override
    public void runOpMode() {
        ComponentTest[] tests = TestDCMotorRunUsingEncoder.Factory.getDCMotorTests(hardwareMap, telemetry);
        waitForStart();
        runTests(tests);
        sleep(20000); // So that there is time to read Telemetry output
    }

    void runTests(ComponentTest[] tests) {
        telemetry.setAutoClear(false);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        Telemetry.Item testStatus = telemetry.addData("Test Status: ", "<span>&#x23F1;</span>");
        testStatus.setRetained(true);

        for (ComponentTest test : tests) {
            try {
                test.init();
                test.runTests();
                while (test.getStatus() == RUNNING) {
                    testStatus.setValue(ComponentTest.Status.getStatusString(RUNNING));
                    telemetry.update();
                }
            } catch (Exception e) {
                testStatus.setValue(ComponentTest.Status.getStatusString(FAILED));
                telemetry.addLine(e.getMessage());
                telemetry.update();
            }
        }
    }
}

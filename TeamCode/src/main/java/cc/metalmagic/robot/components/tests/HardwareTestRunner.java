//package cc.metalmagic.robot.components.tests;
//
//import static cc.metalmagic.robot.components.tests.internal.ComponentTest.Status.FAILED;
//import static cc.metalmagic.robot.components.tests.internal.ComponentTest.Status.RUNNING;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import cc.metalmagic.robot.components.tests.TestDCMotorRunUsingEncoder;
//import cc.metalmagic.robot.components.tests.internal.ComponentTest;
//import cc.metalmagic.robot.components.tests.internal.ComponentTestRunner;
//
//
//@Autonomous(name="Robot Hardware Test Runner", group="Robot")
//public class HardwareTestRunner  extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        ComponentTest[] tests = TestDCMotorRunUsingEncoder.Factory.getDCMotorTests(hardwareMap, telemetry);
//        waitForStart();
//        new ComponentTestRunner(telemetry).runTests(tests);
//        sleep(20000); // So that there is time to read Telemetry output
//    }
//}

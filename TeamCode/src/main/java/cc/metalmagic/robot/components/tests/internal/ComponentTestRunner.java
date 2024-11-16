//package cc.metalmagic.robot.components.tests.internal;
//THIS IS FOR DELETION 9/18
//import static cc.metalmagic.robot.components.tests.internal.ComponentTest.Status.FAILED;
//import static cc.metalmagic.robot.components.tests.internal.ComponentTest.Status.RUNNING;
//
//import androidx.annotation.NonNull;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import cc.metalmagic.robot.components.tests.internal.Annotations.TestDisabled;
//
///***
// * Simple class designed to run tests if they are not disabled.
// * Use it from your OpMode (e.g. {@link cc.metalmagic.robot.components.tests.HardwareTestRunner})
// */
//public class ComponentTestRunner {
//    private final Telemetry telemetry;
//    public ComponentTestRunner(@NonNull Telemetry telemetry){
//        this.telemetry = telemetry;
//    }
//    public void runTests(ComponentTest[] tests) {
//        telemetry.setAutoClear(false);
//        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
//        Telemetry.Item testStatus = telemetry.addData("Test Status: ", "<span>&#x23F1;</span>");
//        testStatus.setRetained(true);
//
//        for (ComponentTest test : tests) {
//            TestDisabled disabled = test.getClass().getAnnotation(TestDisabled.class);
//            if (disabled != null) {
//                continue;
//            }
//
//            try {
//                // Check if Test is Disabled
//                test.init();
//                test.runTests();
//                while (test.getStatus() == RUNNING) {
//                    testStatus.setValue(ComponentTest.Status.getStatusString(RUNNING));
//                    telemetry.update();
//                }
//            } catch (Exception e) {
//                testStatus.setValue(ComponentTest.Status.getStatusString(FAILED));
//                telemetry.addLine(e.getMessage());
//                telemetry.update();
//            }
//        }
//    }
//}

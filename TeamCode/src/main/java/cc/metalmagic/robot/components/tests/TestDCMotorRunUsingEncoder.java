package cc.metalmagic.robot.components.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import cc.metalmagic.robot.HardwareConfig;
import cc.metalmagic.robot.components.tests.Exceptions.TestFailedException;

public class TestDCMotorRunUsingEncoder extends ComponentTest {
    private final DcMotor dcMotor;

    /***
     * This class is designed to run a series of tests on the DC Motor to debug any electrical,
     * mechanical or connection issues
     * @param motor The {@link DcMotor} to be tested.
     */
    public TestDCMotorRunUsingEncoder(DcMotor motor, int portNumber, Telemetry telemetry){
        super(motor.getDeviceName(), portNumber, telemetry);
        dcMotor = motor;
        telemetry.setAutoClear(false);
    }

    private void testForwardMotion(DcMotorSimple.Direction direction) throws TestFailedException {
        int targetPosition = 50*100;
        double tolerance = 2.0/100.0 * targetPosition; //2% tolerance max

        dcMotor.setDirection(direction);
        dcMotor.setTargetPosition(targetPosition);
        dcMotor.setPower(1);

        while (dcMotor.isBusy()){
            // Check if it is running in the right polarity.
            // If its not going in the direction we expect it to,
            // stop the Motor and indicate that the test has failed.
            if (direction == DcMotor.Direction.FORWARD && dcMotor.getCurrentPosition() < 0){
                resetMotor();
                testFailed("\nExpected the Motor to run FORWARD, but found current position to be negative." +
                        "Check if the Motor is plugged in correctly (Polarity is not reversed)\n");
            }

            if (direction == DcMotor.Direction.REVERSE && dcMotor.getCurrentPosition() > 0){
                resetMotor();
                testFailed("\nExpected the Motor to run REVERSE, but found current position to be positive." +
                        "Check if the Motor is plugged in correctly (Polarity is not reversed)\n");
            }

        }
        telemetry.addLine("\nPolarity Test Passed. Checking if it " +
                "reached the intended position in the FORWARD direction" +
                "(with some tolerance for slippages)");
        telemetry.update();

        // Now Check -- did it do approximately 50 Revolutions?
        // It may not be exactly 50 (keep some tolerance),
        // but as long as its in the same direction,
        // that is really what we are testing - check polarity
        int currentPosition  = dcMotor.getCurrentPosition();
        if (Math.abs(targetPosition - currentPosition) > tolerance){
            resetMotor();
            testFailed("\nIncorrect position reached. Expected %d, actual %d. Do you have any " +
                    "mechanical issues such as a roller gone bad, loose or bad motor? It could " +
                    "also be an electrical issue such as low power or bad connection.", targetPosition, dcMotor.getCurrentPosition());
        }
        resetMotor();
    }

    @Override
    void runTestsInternal() throws TestFailedException {
        telemetry.addLine("Resetting " + dcMotor.getDeviceName());
        telemetry.update();
        resetMotor();
        // Run a Test to Check Polarity and Position
        telemetry.addLine("Test Forward Motion");
        telemetry.update();
        testForwardMotion(DcMotor.Direction.FORWARD);

        telemetry.addLine("Test Forward Motion");
        telemetry.update();
        testForwardMotion(DcMotor.Direction.REVERSE);
    }

    @Override
    public int getPortNumber(){
        return dcMotor.getPortNumber();
    }

    private void resetMotor() {
        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcMotor.setPower(0);
    }

    /***
     * A Utility Factory class to create and return DCMotor Tests.
     * If more DC Motors are added and should be tested, you can add it here.
     * Its a simple pattern to make the tests manageable.
     */
    public static class Factory{
        public static ComponentTest[] getDCMotorTests(HardwareMap hardwareMap, Telemetry telemetry){
            // Test Motors
            TestDCMotorRunUsingEncoder frontLeft = new TestDCMotorRunUsingEncoder(
                    hardwareMap.get(DcMotor.class, HardwareConfig.FRONT_LEFT_MOTOR),
                    0, telemetry);
            TestDCMotorRunUsingEncoder frontRight = new TestDCMotorRunUsingEncoder(
                    hardwareMap.get(DcMotor.class, HardwareConfig.FRONT_RIGHT_MOTOR),
                    1, telemetry);
            TestDCMotorRunUsingEncoder backLeft = new TestDCMotorRunUsingEncoder(
                    hardwareMap.get(DcMotor.class, HardwareConfig.BACK_LEFT_MOTOR),
                    2, telemetry);
            TestDCMotorRunUsingEncoder backRight = new TestDCMotorRunUsingEncoder(
                    hardwareMap.get(DcMotor.class, HardwareConfig.BACK_RIGHT_MOTOR),
                    3, telemetry);
            return new ComponentTest[]{frontLeft, frontRight, backLeft, backRight};

        }
    }
}

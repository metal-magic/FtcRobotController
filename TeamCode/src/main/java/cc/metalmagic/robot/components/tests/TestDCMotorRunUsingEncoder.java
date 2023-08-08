package cc.metalmagic.robot.components.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import cc.metalmagic.robot.components.tests.Exceptions.TestFailedException;
import cc.metalmagic.robot.components.tests.Exceptions.TestNotInitializedException;

public class TestDCMotorRunUsingEncoder extends ComponentTest {
    private DcMotor dcMotor;

    /***
     * This class is designed to run a series of tests on the DC Motor to debug any electrical,
     * mechanical or connection issues
     * @param motor
     */
    public TestDCMotorRunUsingEncoder(DcMotor motor, int portNumber, Telemetry telemetry){
        super(motor.getDeviceName(), portNumber, telemetry);
        dcMotor = motor;
        telemetry.setAutoClear(false);
    }

    private void testForwardMotion(DcMotorSimple.Direction direction) throws TestFailedException {
        int targetPosition = 50*100;
        int tolerance = 20;
        dcMotor.setDirection(direction);
        dcMotor.setTargetPosition(targetPosition);
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcMotor.setPower(1);

        while (dcMotor.isBusy()){
            // Check if it is running in the right polarity set above
            if (direction == DcMotor.Direction.FORWARD && dcMotor.getCurrentPosition() < 0){
                // Its not going in the direction we expect it to!
                // Stop the Motor and throw an Exception
                resetMotor();
                testFailed("\nExpected the Motor to run FORWARD, but found current position to be negative." +
                        "Check if the Motor is plugged in correctly (Polarity is not reversed)\n");
            }

            if (direction == DcMotor.Direction.REVERSE && dcMotor.getCurrentPosition() > 0){
                // Its not going in the direction we expect it to!
                // Stop the Motor and throw an Exception
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
                    "mechanical issues such as a roller gone bad, lose motor or bad motor? It could " +
                    "also be an issue with the power or electrical connection.", targetPosition, dcMotor.getCurrentPosition());
        }
        resetMotor();
    }

    @Override
    void runTestsInternal() throws TestNotInitializedException, TestFailedException {
        telemetry.addLine("Resetting " + dcMotor.getDeviceName());
        telemetry.update();
        resetMotor();
        // Run a Test to Check Polarity and Position
        telemetry.addLine("Test Forward Motion");
        telemetry.update();
        testForwardMotion(DcMotor.Direction.FORWARD);

        resetMotor();
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
}

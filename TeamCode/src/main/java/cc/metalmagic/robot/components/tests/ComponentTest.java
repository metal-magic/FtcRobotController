package cc.metalmagic.robot.components.tests;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import cc.metalmagic.robot.components.tests.Exceptions.TestFailedException;
import cc.metalmagic.robot.components.tests.Exceptions.TestNotInitializedException;

/***
 * An abstract class designed to be inherited by all Tests. See an example implementation
 * {@link TestDCMotorRunUsingEncoder}
 */
public abstract class ComponentTest {
    public enum Status {
        /***
         * Status is inan unknown condition. Usually it means it has
         * not been {@link Status#INITIALIZED}.
         */
        Unknown,

        /***
         * The test is Initialized Properly. If not, the test will fail to start.
         */
        INITIALIZED,

        /***
         * Test is running. Usually that means if you try to run it again, it is invalid.
         */
        RUNNING,

        /*** Test is finished running with or without throwing
         * @see TestFailedException
         * DONOT attempt to restart the test until it is {@link Status#INITIALIZED} again.
         */
        DONE,

        /*** Test is cancelled while it was running.
         * DONOT attempt to restart the test until it is {@link Status#INITIALIZED} again.
         */
        CANCELLED
    }
    private int port;
    protected Telemetry telemetry;
    private Status status = Status.Unknown;
    private String testName;
    public ComponentTest(String testName, int portNumber, @NonNull Telemetry telemetry){
        port = portNumber;
        this.telemetry = telemetry;
        this.testName = testName;
    }
    /***
     * Initialize the Test, otherwise it will fail to run
     * @param portNumber The Port number that you think the device is connected to.
     *                   The real port number is queried at runtime to make sure that
     *                   this port number matches the expectation.
     * @param telemetry
     */
    public final void init(int portNumber, @NonNull Telemetry telemetry){
        // RESERVING FOR FUTURE IN CASE WE NEED TO DO ANY further Initialization
        status = Status.INITIALIZED;
    }

    /***
     * Internal method to initialize the state machine and run some basic tests (e.g. check port number)
     * @throws Exception
     */
    abstract void runTestsInternal() throws TestNotInitializedException, TestFailedException;

    final void testFailed(String format, Object... args) throws TestFailedException{
        String errorMessage = String.format(format, args);
        setStatus(Status.DONE);
        throw new TestFailedException(testName, errorMessage);
    }

    /***
     * Put all your tests under this method.
     * @throws TestFailedException if a test fails.
     */
    public final void runTests() throws TestNotInitializedException, TestFailedException{
        if (status != Status.INITIALIZED) {
            throw new TestNotInitializedException(testName, "The Test must be initialized.");
        }
        setStatus(Status.RUNNING);

        if (getPortNumber() == port){
            testFailed("Incorrect Por Number. Expected %d, got %d. Please check your Connection."
                    ,port, getPortNumber());
        }

        runTestsInternal();
        // If we reached here, no errors were encountered. We can set the state to "DONE"
        setStatus(Status.DONE);
    }

    /***
     * Get the port number that the device is connected to.
     * Note that this is queried from the hardware. The primary
     * purpose is to check if the connection to the Hub or Controller
     * is what you think it is.
     * @return The actual Port Number of the device connection.
     */
    public abstract int getPortNumber();

    /***
     * @return the current Status of the Test
     */
    public final Status getStatus(){
        return status;
    }

    /***
     * Internal method designed for sub-classes (Tests) to set the status of the
     * running test.
     * @param status
     */
    final void setStatus(Status status){
        this.status = status;
    }
}

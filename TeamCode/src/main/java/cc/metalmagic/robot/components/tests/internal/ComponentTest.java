//package cc.metalmagic.robot.components.tests.internal;
//
//import androidx.annotation.NonNull;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import cc.metalmagic.robot.components.tests.internal.Exceptions.TestFailedException;
//import cc.metalmagic.robot.components.tests.internal.Exceptions.TestNotInitializedException;
//import cc.metalmagic.robot.components.tests.TestDCMotorRunUsingEncoder;
//
///***
// * An abstract class designed to be inherited by all Tests. See an example implementation
// * {@link TestDCMotorRunUsingEncoder}
// */
//public abstract class ComponentTest {
//    public enum Status {
//        /***
//         * Status is in an unknown condition. Usually it means it has
//         * not been {@link Status#INITIALIZED}.
//         */
//        Unknown,
//
//        /***
//         * The test is Initialized Properly. If not, the test will fail to start.
//         */
//        INITIALIZED,
//
//        /***
//         * Test is running. Usually that means if you try to run it again, it is invalid.
//         */
//        RUNNING,
//
//        /*** Test is finished running and is successful.
//         * @see TestFailedException
//         * DONOT attempt to restart the test until it is {@link Status#INITIALIZED} again.
//         */
//        DONE,
//        /*** Test is finished, but failed
//         *
//         */
//        FAILED,
//
//        /*** Test is cancelled while it was running.
//         * DO NOT attempt to restart the test until it is {@link Status#INITIALIZED} again.
//         */
//        CANCELLED;
//
//        public static String getStatusString(Status status){
//            switch (status){
//                case DONE: return "<span>&#9989;</span>";
//                case RUNNING: return "<span>&#9201;</span>";
//                case CANCELLED: return "<span>&#128683;</span>";
//                case FAILED: return "<span'>&#10060;</span>";
//                default: return "<span>&#10067;</span>";
//            }
//        }
//    }
//    private final int port;
//    protected Telemetry telemetry;
//    private Status status = Status.Unknown;
//    private final String testName;
//
//    /***
//     * Create a Test
//     * @param testName Give your test a name that has some meaning (e.g. Left Motor Test)
//     * @param portNumber The Port number that you think the device is connected to.
//     *      *                   The real port number is queried at runtime to make sure that
//     *      *                   this port number matches the expectation.
//     * @param telemetry {@link Telemetry} to display status on the Driver Station.
//     */
//    public ComponentTest(String testName, int portNumber, @NonNull Telemetry telemetry){
//        port = portNumber;
//        this.telemetry = telemetry;
//        this.testName = testName;
//    }
//    /***
//     * Initialize the Test, otherwise it will fail to run
//     */
//    public final void init(){
//        // RESERVING FOR FUTURE IN CASE WE NEED TO DO ANY further Initialization
//        status = Status.INITIALIZED;
//    }
//
//    /***
//     * Override this method and run your tests here.
//     * @throws TestFailedException If the test fails. Using Exceptions as a way to force checking the status
//     * and stop the tests and perform any cleanup (e.g. resetting Component).
//     */
//    protected abstract void runTestsInternal() throws TestFailedException;
//
//    protected final void testFailed(String format, Object... args) throws TestFailedException{
//        String errorMessage = String.format(format, args);
//        setStatus(Status.FAILED);
//        throw new TestFailedException(testName, errorMessage);
//    }
//
//    /***
//     * A general method which sets up the state properly, performs some basic checks and calls
//     * @see ComponentTest#runTestsInternal() . You should put your tests there (after overriding it
//     * @throws TestNotInitializedException If the test has not been initialized {@link ComponentTest#init()}
//     * @throws TestFailedException If the test fails. Using Exceptions as a way to force checking the status
//     * and stop the tests and perform any cleanup (e.g. resetting the Component.
//     */
//    public final void runTests() throws TestNotInitializedException, TestFailedException{
//        if (status != Status.INITIALIZED) {
//            throw new TestNotInitializedException(testName, "The test must be initialized before running it.");
//        }
//
//        setStatus(Status.RUNNING);
//
//        if (getPortNumber() == port){
//            testFailed("Incorrect Port Number. Expected %d, got %d. Please check your Connection."
//                    ,port, getPortNumber());
//        }
//
//        runTestsInternal();
//        // If we reached here, no errors were encountered. We can set the state to "DONE"
//        setStatus(Status.DONE);
//    }
//
//    /***
//     * Get the port number that the device is connected to.
//     * Note that this is queried from the hardware. The primary
//     * purpose is to check if the connection to the Hub or Controller
//     * is what you think it is.
//     * @return The actual Port Number of the device connection.
//     */
//    public abstract int getPortNumber();
//
//    /***
//     * @return the current Status of the Test
//     */
//    public final Status getStatus(){
//        return status;
//    }
//
//    /***
//     * Internal method designed for sub-classes (Tests) to set the status of the
//     * running test.
//     * @param status {@link ComponentTest.Status}
//     */
//    final void setStatus(Status status){
//        this.status = status;
//    }
//}

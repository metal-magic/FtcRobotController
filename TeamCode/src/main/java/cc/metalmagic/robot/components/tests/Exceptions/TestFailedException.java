package cc.metalmagic.robot.components.tests.Exceptions;

public class TestFailedException extends TestException{
    public TestFailedException(String testName, String message){
        super(testName, message);
    }
}

package cc.metalmagic.robot.components.tests.internal.Exceptions;

public class TestNotInitializedException extends TestException{
    public TestNotInitializedException(String testName, String message){
        super(testName, message);
    }
}

package cc.metalmagic.robot.components.tests.internal.Exceptions;

public class TestException extends  Exception {
    public TestException(String testName, String message){
        super(testName + "failed. Message:" + message);
    }
}

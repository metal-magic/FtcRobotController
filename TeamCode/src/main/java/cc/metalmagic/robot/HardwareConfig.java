package cc.metalmagic.robot;

/***
 * A Constants file to put the names of various Sensors and Components in one place.
 * For example, if you are going to change the name of a motor in the Robot Controller
 * configuration, change the corresponding string here.
 */
public final class HardwareConfig {
    private HardwareConfig(){
        // Constructor is private to prevent objects from being created.
        // This class is designed to "hold" constants in one place.
    }
    public static final String FRONT_LEFT_MOTOR = "motorFrontLeft";
    public static final String FRONT_RIGHT_MOTOR = "motorFrontRight";
    public static final String BACK_LEFT_MOTOR = "motorBackLeft";
    public static final String BACK_RIGHT_MOTOR = "motorBackRight";
}

package cc.metalmagic.robot;

/***
 * Class designed to hold common constants & constant calculations in a central place.
 */
public final class Constants {
    private Constants(){
        // Constructor is private to prevent objects from being created.
        // This class is designed to "hold" constants in one place.
    }
    /*
     goBILDA 5203 series Yellow Jacket
     The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    */

    public static final double MOTOR_TICK_COUNT = 537.7;
    public static final double WHEEL_DIAMETER_MM = 96;
    public static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4; // in Inches
    public static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches
}

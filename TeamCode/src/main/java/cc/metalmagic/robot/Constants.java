package cc.metalmagic.robot;

public abstract class Constants {
    // goBILDA 5203 series Yellow Jacket
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4

    public static final double MOTOR_TICK_COUNTS = 537.7;
    public static final double WHEEL_DIAMETER_INCHES = 96 / 25.4; // in Inches
    public static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches
}

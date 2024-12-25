package org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

public interface Localizer {
    Twist2dDual<Time> update();
}

//package org.firstinspires.ftc.teamcode.mmintothedeep.util.ControlTheory;
//
//import java.lang.reflect.Array;
//import java.util.ArrayList;
//
//public class testSCurve {
//
////    def s_curve_motion_profile(start_pos, end_pos, max_vel, max_acc, total_time):
////            """
////    Generates an S-curve motion profile with given start and end positions, maximum velocity, acceleration, and total time.
////
////    Args:
////        start_pos (float): Initial position.
////        end_pos (float): Final position.
////        max_vel (float): Maximum velocity.
////        max_acc (float): Maximum acceleration.
////        total_time (float): Total time for the movement.
////
////    Returns:
////        list: A list of positions at each time step.
////    """
////
////    distance = end_pos - start_pos  # Total distance to travel
////
////    # Calculate time for acceleration and deceleration phases
////    t_acc = max_vel / max_acc
////            t_dec = max_vel / max_acc
////    t_const_vel = total_time - 2 * t_acc - 2 * t_dec  # Time at constant velocity
////
////    # Generate position data for each phase
////    acc_phase = [start_pos + 0.5 * max_acc * t**2 for t in range(int(t_acc))]
////    const_vel_phase = [start_pos + max_vel * t for t in range(int(t_const_vel))]
////    dec_phase = [end_pos - 0.5 * max_acc * t**2 for t in range(int(t_dec))]
////
////            # Combine phases
////    positions = acc_phase + const_vel_phase + dec_phase
////
////    return positions
//    public double[] s_curve_motion_profile(double start_pos, double end_pos, double max_vel, double max_acc, double total_time) {
//        double distance = end_pos - start_pos; // Total distance to travel
//
//        // Calculate time for acceleration, constant velocity, and deceleration phases
//        double t_acc = max_vel / max_acc; // Time spent in the acceleration phase - Vel = Acceleration * Time
//        double t_dec = max_vel / max_acc; // Almost always equal to "t_acc" but this is for full control
//        double t_const_vel = total_time - t_acc - t_dec; // Time spent at constant velocity
//
//        // Generate position data for each phase
//        ArrayList<Double> acc_phase = // ArrayList of positions for the acceleration phase
//    }
//}

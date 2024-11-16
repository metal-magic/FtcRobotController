//package org.firstinspires.ftc.teamcode.mmcenterstage.other;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//public class DistanceSensCode extends LinearOpMode {
//
//    private DistanceSensor distanceSensor;
//
//    @Override
//    public void runOpMode() {
//        // Initialize distance sensor
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
//
//        // Wait for start button to be pressed
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Get distance measured by the sensor
//            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
//
//            // Print distance to the telemetry
//            telemetry.addData("Distance (inches)", distance);
//            telemetry.update();
//
//            // Add a small delay to reduce loop frequency
//            sleep(100);
//        }
//    }
//}

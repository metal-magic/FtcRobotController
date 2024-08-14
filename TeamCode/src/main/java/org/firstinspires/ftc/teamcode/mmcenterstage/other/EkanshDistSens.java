package org.firstinspires.ftc.teamcode.mmcenterstage.other;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous


public class EkanshDistSens extends LinearOpMode {


    private DistanceSensor distanceSensor;


    @Override
    public void runOpMode() {
        // Initialize distance sensor
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");
        // Wait for start button to be pressed
        waitForStart();


        while (opModeIsActive()) {
            // Get distance measured by the sensor
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
            boolean b = (distance <= 3);
            while (b){
                distance = distanceSensor.getDistance(DistanceUnit.INCH);
                leftBackDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                telemetry.addData("Distance (inches)", distance);
                telemetry.update();
            }
            // Print distance to the telemetry
            telemetry.addData("Distance (inches)", distance);
            telemetry.update();


            // Add a small delay to reduce loop frequency
            sleep(100);
        }
    }
}

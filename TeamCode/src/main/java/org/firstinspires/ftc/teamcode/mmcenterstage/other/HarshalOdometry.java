package org.firstinspires.ftc.team1234; // Replace with your own package name

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HarshalOdometry extends LinearOpMode {
    // Constants
    private static final double TICKS_PER_REV = 8192.0; // Replace with your encoder ticks per revolution
    private static final double WHEEL_DIAMETER = 2.0; // Replace with your wheel diameter in inches
    private static final double TRACK_WIDTH = 14.0; // Replace with your track width in inches

    // Motors
    private DcMotorEx leftEncoder, rightEncoder, lateralEncoder;

    // Odometry variables
    private double prevLeftTicks, prevRightTicks, prevLateralTicks;
    private Pose2d poseEstimate = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() {
        // Initialize hardware
        HardwareMap hardwareMap = new HardwareMap();
        leftEncoder = hardwareMap.get(DcMotorEx.class, "left_encoder");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "right_encoder");
        lateralEncoder = hardwareMap.get(DcMotorEx.class, "lateral_encoder");

        // Set motor directions if needed
        leftEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        rightEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        lateralEncoder.setDirection(DcMotorEx.Direction.FORWARD);

        // Initialize encoders
        leftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lateralEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lateralEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            // Update encoder ticks
            RevBulkData bulkData = new RevBulkData();
            bulkData.populate(leftEncoder, rightEncoder, lateralEncoder);
            double leftTicks = bulkData.getMotorCurrentPosition(leftEncoder);
            double rightTicks = bulkData.getMotorCurrentPosition(rightEncoder);
            double lateralTicks = bulkData.getMotorCurrentPosition(lateralEncoder);

            // Calculate wheel displacements
            double leftDeltaTicks = leftTicks - prevLeftTicks;
            double rightDeltaTicks = rightTicks - prevRightTicks;
            double lateralDeltaTicks = lateralTicks - prevLateralTicks;

            double leftDistance = leftDeltaTicks / TICKS_PER_REV * WHEEL_DIAMETER;
            double rightDistance = rightDeltaTicks / TICKS_PER_REV * WHEEL_DIAMETER;
            double lateralDistance = lateralDeltaTicks / TICKS_PER_REV * WHEEL_DIAMETER;

            // Update pose estimate
            poseEstimate = poseEstimate.plus(Pose2d.fromVector(
                    (leftDistance + rightDistance) / 2.0,
                    lateralDistance,
                    (rightDistance - leftDistance) / TRACK_WIDTH
            ));

            // Store previous ticks
            prevLeftTicks = leftTicks;
            prevRightTicks = rightTicks;
            prevLateralTicks = lateralTicks;

            // Display pose estimate
            telemetry.addData("X", poseEstimate.getX());
            telemetry.addData("Y", poseEstimate.getY());
            telemetry.addData("Heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}


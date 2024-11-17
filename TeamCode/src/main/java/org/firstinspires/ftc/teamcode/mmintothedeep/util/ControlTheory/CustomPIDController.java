package org.firstinspires.ftc.teamcode.mmintothedeep.util.ControlTheory;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class CustomPIDController extends LinearOpMode {
    DcMotor testMotor;

    double integral = 0;
    double repetitions = 0;

    public static PIDCoefficients testPID = new PIDCoefficients(0.3,0,0);

    FtcDashboard dashboard;

    public static double TARGET_POS = 1000; // 1000 is default value

    ElapsedTime PIDTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        testMotor = hardwareMap.dcMotor.get("linearSlideMotor");

        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        moveTestMotor(TARGET_POS);

    }
    void moveTestMotor(double targetPosition) {
        double error = testMotor.getCurrentPosition();
        double lastError = 0;
        double up;

        /*
         * Comparison value dependent on motor tick count
         * Higher end motor tick count: higher value
         * Lower end motor tick count: lower value
         */
        while (Math.abs(error) <= 9 /*Modify with above comments*/ && repetitions < 400 /*Modify*/) {
            error = testMotor.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
            integral += changeInError * PIDTimer.time();
            double derivative = changeInError / PIDTimer.time();
            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            if (testMotor.getCurrentPosition() < 0) { // Set Minimum
                testMotor.setPower(0.3);
            }
            else if (testMotor.getCurrentPosition() > 3200) {
                testMotor.setPower(-0.3);
            }
            else {
                testMotor.setPower(P + I + D);
            }
            error = lastError;
            PIDTimer.reset();
            repetitions++;
            telemetry.addData("Reference", TARGET_POS);
            telemetry.addData("Response", testMotor.getCurrentPosition());
            telemetry.addData("Command", testMotor.getCurrentPosition()+testMotor.getPower());
        }
    }
}
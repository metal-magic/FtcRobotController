package org.firstinspires.ftc.teamcode.mmcenterstage.HardwareTesting;

import android.icu.util.DateInterval;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.time.Instant;
import java.util.Date;

@TeleOp
@Disabled
public class PivotServoTest  extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Gamepad currentStateForScorerGamePad = new Gamepad();
        // Gamepad previousStateForScorerGamePad = new Gamepad();
        Servo pivotServo = hardwareMap.servo.get("pivotServo");
        ((ServoImplEx)pivotServo).setPwmRange(new PwmControl.PwmRange(500, 2500));

        Date previousTime = new Date();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            // pivotServoPos = pivotServo.getPosition();

            // previousStateForScorerGamePad.copy(currentStateForScorerGamePad);

            // Store the current state for future iteration
            // currentStateForScorerGamePad.copy(gamepad2);

            Date currentTime = new Date();

            if(currentTime.getTime() - previousTime.getTime() > 100)
            {
                double moveIncrement = 0.05;
                if(gamepad2.x) {
                    moveIncrement = 0.01;
                }else{
                    moveIncrement =  0.05;
                }
                if (gamepad2.dpad_up) {
                    telemetry.addLine("Servo Will go Up");
                    pivotServo.setPosition(pivotServo.getPosition() + moveIncrement);
                }


                if (gamepad2.dpad_down) {
                    telemetry.addLine("Servo Will go down");
                    pivotServo.setPosition(pivotServo.getPosition() - moveIncrement);
                }
                previousTime = currentTime;
            }

        }

    }
}

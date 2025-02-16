package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.ForCompetition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

@TeleOp(name = "!! Test The Specimen Stuff")
public class SpecClawTest extends LinearOpMode {

    public Servo pivotServo = null;
    public Servo clipServo = null;

    public void runOpMode() {
        initialize();

        while (opModeIsActive()) {
            runLoop();
        }
    }

    public void initialize() {
        pivotServo = hardwareMap.servo.get("specPivot");
        clipServo = hardwareMap.servo.get("clipServo");

        telemetry.addLine("initialized");
    }

    public void runLoop() {
        // clip servo open and close
        if (gamepad1.right_bumper) {
            clipServo.setPosition(UtilityValues.CLIP_POS_CLOSE);
        }
        if (gamepad1.left_bumper) {
            clipServo.setPosition(UtilityValues.CLIP_POS_OPEN);
        }

        if (gamepad1.y) {
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_DOWN);
        }

        // pivot specimen
        if (gamepad1.dpad_up) {
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_UP);
        }
        if (gamepad1.dpad_down) {
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_SCORE);
            sleep(200);
            clipServo.setPosition(UtilityValues.CLIP_POS_OPEN);
            pivotServo.setPosition(UtilityValues.SPECIMEN_PIVOT_SCORE);
        }
    }

}

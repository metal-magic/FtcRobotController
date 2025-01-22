package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.partsTest;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.Date;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "linear Slide Test2", group = "Autonomous")
@Disabled
public class LinearSlide2 extends LinearOpMode {
    Date currentTime = new Date();

    public DcMotor linearSlideMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        initMotor();

        waitForStart();

        /*
         * ============================
         * THIS IS THE ACTUAL DRIVING
         * ============================
         */

        /*
         * METAL MAGIC INTO THE DEEP
         * THIS CODE STARTS ON THE LEFT SIDE OF THE BLUE SIDE (closer to backdrop)
         * SCORES SAMPLE AND PARKS IN CORNER
         * THIS IS A TEST FILE TO TEST AUTONOMOUS CODE TO BE EVENTUALLY USED
         */
        // sleep lines are to avoid two lines of codes running at the same time

        while (opModeIsActive() && !isStopRequested()) {
            moveLinearSlide(600, 0.1);
        }

    }

    public void initMotor() {
        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");

        linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * while (linearSlideMotor.getCurrentPosition() > 0) {
         * linearSlideMotor.setPower(-0.5);
         * }
         * while (linearSlideMotor.getCurrentPosition() < 0) {
         * linearSlideMotor.setPower(0.3);
         * }
         */

        // ABOVE THIS, THE ENCODERS AND MOTOR ARE NOW RESET
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void moveLinearSlide(int height, double power) {
        double scale = power;
        // Checks if current position is within bounds
        if (linearSlideMotor.getCurrentPosition() < 4000 && height > linearSlideMotor.getCurrentPosition()) {
            while (height > linearSlideMotor.getCurrentPosition()) {
                if (linearSlideMotor.getCurrentPosition() < 3000) {
                    linearSlideMotor.setPower(scale);
                } else {
                    linearSlideMotor.setPower(scale * 0.5);
                }
            }
        } else if (linearSlideMotor.getCurrentPosition() > 50 && height < linearSlideMotor.getCurrentPosition()) {
            while (height < linearSlideMotor.getCurrentPosition()) {
                if (linearSlideMotor.getCurrentPosition() > 1000) {
                    linearSlideMotor.setPower(-scale);
                } else {
                    linearSlideMotor.setPower(-scale * 0.5);
                }
            }
        } else {
            linearSlideMotor.setPower(0);
        }
    }
}
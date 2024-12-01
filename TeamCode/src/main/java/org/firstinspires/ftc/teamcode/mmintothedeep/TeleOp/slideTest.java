package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mmcenterstage.HardwareTesting.LeftStrafeTest;
import org.firstinspires.ftc.teamcode.mmcenterstage.other.OldSensorColor2;
import org.firstinspires.ftc.teamcode.mmintothedeep.util.UtilityValues;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Date;
import java.util.Objects;
import java.util.concurrent.TimeUnit;
import java.util.Timer;
@TeleOp
public class slideTest extends OpMode {
    public DcMotor linearSlideMotor = null;
    boolean isPressed = false;
    @Override
    public void init() {
        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper) {
            linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(-0.2*0.41);
        } else if (gamepad2.left_bumper) {
            linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(0.2*0.41);
        } else {
            linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linearSlideMotor.setPower(0);
        }
        telemetry.addData("Slide position, ", linearSlideMotor.getCurrentPosition());

        if (gamepad2.dpad_left) {
            isPressed = false;
        }
        // torque 117 rpm
//        if (gamepad2.dpad_right) {
//            isPressed = true;
//        } else if (isPressed == true) {
//            while (linearSlideMotor.getCurrentPosition() < -50) {
//                linearSlideMotor.setPower(0.2);
//            }
//            linearSlideMotor.setPower(0);
//        }

        if (gamepad2.dpad_right) {
            isPressed = true;
        } else if (isPressed == true) {
            while (linearSlideMotor.getCurrentPosition() >= 50) {
                linearSlideMotor.setPower(-0.41*0.2);
                telemetry.addData("pojijon", linearSlideMotor.getCurrentPosition());
                telemetry.update();
                if (linearSlideMotor.getCurrentPosition() < 65) {
                    telemetry.addData("pojijonwithrizz", linearSlideMotor.getCurrentPosition());
                    telemetry.update();
                    isPressed = false;
                    break;
                }
            }
            linearSlideMotor.setPower(0);
        }
    }
}

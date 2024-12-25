package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.partsTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * =======================================================================
 *
 * OP MODE TO TEST THE LINEAR SLIDE
 * Please read:
 * 1. In configuration, name the servo as 'testSlide'
 * 2. Find the TeleOp that is called: 'Linear Slide Test'
 * GAMEPAD 1:
 * 3. left trigger -> move slide down
 * 4. right trigger -> move slide up
 * 5. Controls may vary based on motor orientation, sometimes it may be flipped
 * 6. Slide motor position is printed as telemetry. Use this to find limits
 *
 * =======================================================================
 */

@TeleOp(group = "Test Parts", name = "Linear Slide Test")
public class LinearSlideTest extends OpMode {

    public DcMotor testSlide = null;

    float slideSpeed = 0; // set speed

    @Override
    public void init() {
        testSlide = hardwareMap.dcMotor.get("testSlide");
        testSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {

        telemetry.update();

        // setting speed based on controls
        slideSpeed = gamepad1.right_trigger - gamepad1.left_trigger;

        testSlide.setPower(slideSpeed);

        // to set limits
        telemetry.addData("Slide Motor Position", testSlide.getCurrentPosition());

    }

}

package org.firstinspires.ftc.teamcode.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmMotorTest extends OpMode {

    DcMotor armMotor = null;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

    }

    @Override
    public void loop() {
        armMotor.setPower(gamepad2.left_stick_y);
    }


}

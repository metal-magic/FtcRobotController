package org.firstinspires.ftc.teamcode.mmintothedeep.TeleOp.partsTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "idk slideTest", group = "parts")
@Disabled
public class slideTest extends OpMode {
    public DcMotor hangSlideMotor = null;
    public DcMotor hangSlideMotor2 = null;
    boolean isPressed = false;
    boolean isPressed2 = false;

    boolean isPressedEndOHYE = false;
    @Override
    public void init() {
        hangSlideMotor = hardwareMap.dcMotor.get("hangSlideMotor1");
        hangSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangSlideMotor2 = hardwareMap.dcMotor.get("hangSlideMotor2");
        hangSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {


        if (gamepad2.dpad_left) {
            isPressed = false;
        }
        // torque 117 rpm
//        if (gamepad2.dpad_right) {
//            isPressed = true;
//        } else if (isPressed == true) {
//            while (hangSlideMotor.getCurrentPosition() < -50) {
//                hangSlideMotor.setPower(0.2);
//            }
//            hangSlideMotor.setPower(0);
//        }

        if (gamepad2.dpad_right) {
            isPressed = true;
        } else if (isPressed == true) {
            while (hangSlideMotor.getCurrentPosition() >= 50) {
                hangSlideMotor.setPower(0.41*0.2);
                telemetry.addData("pojijon", hangSlideMotor.getCurrentPosition());
                telemetry.update();
                if (hangSlideMotor.getCurrentPosition() < 65) {
                    telemetry.addData("pojijonwithrizz", hangSlideMotor.getCurrentPosition());
                    telemetry.update();
                    isPressed = false;
                    break;
                }
            }
            hangSlideMotor.setPower(0);
        }

        if (gamepad2.right_bumper) {
            hangSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hangSlideMotor.setPower(0.2);
        } else if (gamepad2.left_bumper) {
            if (hangSlideMotor.getCurrentPosition() > -4338) {
                hangSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                hangSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hangSlideMotor.setPower(-0.2);
            }
        } else {
            if (!isPressedEndOHYE) {
                hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hangSlideMotor.setPower(0);
            }
        }
        telemetry.addData("Slide position, ", hangSlideMotor.getCurrentPosition());

        if (gamepad2.right_trigger>=0.3F) {
            hangSlideMotor2.setDirection(DcMotor.Direction.FORWARD);
            hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hangSlideMotor2.setPower(-0.2*0.41);
        } else if (gamepad2.left_trigger>=0.3F) {
            if (hangSlideMotor.getCurrentPosition() > -4338) {
                hangSlideMotor2.setDirection(DcMotor.Direction.FORWARD);
                hangSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hangSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hangSlideMotor2.setPower(0.2 * 0.41);
            }
        } else {
            if (!isPressedEndOHYE) {
                hangSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hangSlideMotor2.setPower(0);
            }
        }


        if (gamepad2.dpad_up) {
            isPressedEndOHYE = true;
        } else {
            if (isPressedEndOHYE) {
                hangSlideMotor.setPower(0.2);
                hangSlideMotor2.setPower(-0.2*0.41);
            }
        }


        if (gamepad2.dpad_left) {
            isPressed2 = true;
        } else if (isPressed2 == true) {
            while (hangSlideMotor2.getCurrentPosition() >= 50) {
                hangSlideMotor2.setPower(-0.41*0.2);
                telemetry.addData("pojijon2", hangSlideMotor2.getCurrentPosition());
                telemetry.update();
                if (hangSlideMotor2.getCurrentPosition() < 65) {
                    telemetry.addData("pojijonwithrizz2", hangSlideMotor2.getCurrentPosition());
                    telemetry.update();
                    isPressed = false;
                    break;
                }
            }
            hangSlideMotor2.setPower(0);
        }
    }

}

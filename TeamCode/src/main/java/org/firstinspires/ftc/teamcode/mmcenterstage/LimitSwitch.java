package org.firstinspires.ftc.teamcode.mmcenterstage;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class LimitSwitch extends OpMode {

    DigitalChannel Limitswitch = new DigitalChannel() {
        @Override
        public Mode getMode() {
            return null;
        }

        @Override
        public void setMode(Mode mode) {

        }

        @Override
        public boolean getState() {
            return getState();
        }

        @Override
        public void setState(boolean state) {

        }

        @Override
        public void setMode(DigitalChannelController.Mode mode) {

        }

        @Override
        public Manufacturer getManufacturer() {
            return null;
        }

        @Override
        public String getDeviceName() {
            return null;
        }

        @Override
        public String getConnectionInfo() {
            return null;
        }

        @Override
        public int getVersion() {
            return 0;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {

        }

        @Override
        public void close() {

        }
    };

    @Override
    public void init() {
        CRServo armMotor;
    }

    @Override
    public void loop() {
        CRServo armMotor = hardwareMap.crservo.get("armMotor");
        armMotor.setDirection(CRServo.Direction.REVERSE);
        armMotor.setPower(gamepad2.left_stick_y);
    if (Limitswitch.getState()) {
        armMotor.setPower(0);
    }
    }
}

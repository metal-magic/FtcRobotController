package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


public class MetalMagicExternalHardware {

    private IMU imu = null;
    public void runOpMode(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {

        initIMU(hardwareMap, telemetry);
    }

    private void initIMU(HardwareMap hardwareMap, Telemetry telemetry) {
//        BHI260IMU.
//        IMU.Parameters parameters = new BHI260IMU.Parameters(RevHubOrientationOnRobot.LogoFacingDirection.);
//        parameters.angleUnit = BHI260IMU.a.DEGREES;
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);


    }


}

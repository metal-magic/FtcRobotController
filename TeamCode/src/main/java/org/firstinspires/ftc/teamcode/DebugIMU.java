package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="Debug IMU", group="Debug")
public class DebugIMU extends LinearOpMode {
    private IMU imu = null;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        initIMU();
    }

    private void initIMU() {
//        BHI260IMU.
//        IMU.Parameters parameters = new BHI260IMU.Parameters(RevHubOrientationOnRobot.LogoFacingDirection.);
//        parameters.angleUnit = BHI260IMU.a.DEGREES;
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(orientation));

        telemetry.addLine()
                .addData("IMU FirstAngle", imu.getRobotOrientation(
                        AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).firstAngle)
                .addData("IMU SecondAngle", imu.getRobotOrientation(
                        AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).secondAngle)
                .addData("IMU ThirdAngle", imu.getRobotOrientation(
                        AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle)
                .addData("Device Name", "%s", imu.getDeviceName());
        telemetry.update();
        sleep(30000);

    }
}

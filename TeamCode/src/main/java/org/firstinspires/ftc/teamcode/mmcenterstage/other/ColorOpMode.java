package org.firstinspires.ftc.teamcode.mmcenterstage.other;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous()
@Disabled
public class ColorOpMode extends OpMode {
    SensorColor2 board = new SensorColor2();

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("RGB is ", board.getRGB());
        telemetry.addData("Proximity is ", board.getProximity(DistanceUnit.INCH)  );
    }
}

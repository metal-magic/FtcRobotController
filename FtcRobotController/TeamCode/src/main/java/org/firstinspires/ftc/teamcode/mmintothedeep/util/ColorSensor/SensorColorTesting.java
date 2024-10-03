package org.firstinspires.ftc.teamcode.mmintothedeep.util.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous()
public class SensorColorTesting extends OpMode {
    SensorColor board = new SensorColor();

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("RGB is ", board.getRGB());
        if (board.getBlue() > 4000) {
            telemetry.addLine("Object is blue");
        }
        else if (board.getRed() > 1000) {
            telemetry.addLine("Object is red");
        }
        telemetry.addData("Proximity is ", board.getProximity(DistanceUnit.INCH)  );
    }
}

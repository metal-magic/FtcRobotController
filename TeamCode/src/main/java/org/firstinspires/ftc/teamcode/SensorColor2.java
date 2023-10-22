package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class SensorColor2 {
    private ColorSensor colorSensor;
    public void init(HardwareMap hwMap)  {
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
    }
    public String getRGB() {
        return "(" + colorSensor.red() + " , " + colorSensor.green() + " , " + colorSensor.blue() + ")";
    }


}

package org.firstinspires.ftc.teamcode.mmcenterstage;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SensorColor2 {
    private ColorSensor colorSensor;
    public void init(HardwareMap hwMap)  {
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
    }
    public String getRGB() {
        return "(" + colorSensor.red() + " , " + colorSensor.green() + " , " + colorSensor.blue() + ")";
    }


}

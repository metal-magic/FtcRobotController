package org.firstinspires.ftc.teamcode.mmcenterstage.other;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
public class OldSensorColor2 {
    private ColorSensor colorSensor;
    private DistanceSensor proximitySensor;
    public void init(HardwareMap hwMap)  {
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        proximitySensor = hwMap.get(DistanceSensor.class, "proximitySensor");
    }
    public String getRGB() {
        return "(" + (colorSensor.red()) + " , " + colorSensor.green() + " , " + colorSensor.blue() + ")";
    }
    public String getProximity(DistanceUnit du) {
        return proximitySensor.getDistance(du)+" inches";
    }

}

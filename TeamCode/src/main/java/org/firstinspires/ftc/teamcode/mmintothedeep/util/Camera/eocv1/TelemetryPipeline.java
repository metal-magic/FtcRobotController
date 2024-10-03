package org.firstinspires.ftc.teamcode.mmintothedeep.util.Camera.eocv1;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryPipeline extends OpenCvPipeline {

    Telemetry telemetry;

    public TelemetryPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        telemetry.addData("[Hello]", "World!");
        telemetry.update();
        return input; // Return the input mat
    }

}
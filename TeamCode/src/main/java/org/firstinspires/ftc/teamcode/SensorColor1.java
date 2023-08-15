/* Copyright (c) 2017-2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Autonomous color sensor
 */
@Disabled
@Autonomous(name = "Sensor: Color", group = "Sensor")

public class SensorColor1 extends LinearOpMode {


  /**
   * The runOpMode() method is the root of this Op Mode, as it is in all LinearOpModes.
   * Our implementation here, though is a bit unusual: we've decided to put all the actual work
   * in the runSample() method rather than directly in runOpMode() itself. The reason we do that is
   * that in this sample we're changing the background color of the robot controller screen as the
   * Op Mode runs, and we want to be able to *guarantee* that we restore it to something reasonable
   * and palatable when the Op Mode ends. The simplest way to do that is to use a try...finally
   * block around the main, core logic, and an easy way to make that all clear was to separate
   * the former from the latter in separate methods.
   */
  private ColorSensor colorSensor;
  private double redValue;
  private double greenValue;
  private double blueValue;


  @Override
  public void runOpMode() throws InterruptedException {
    //calling methods that we made below
    initColorSensor();
    while (!isStarted()) {
      getColor();
      colorTelemetry();
    }
    waitForStart();
    while (opModeIsActive()) {
    getColor();
    colorTelemetry();
    }
  }
  public void initColorSensor() {
    //Telling the sensor to detect the color in front of it
    colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
  }
  public void getColor() {
    //getting the results of the sensor
    redValue = colorSensor.red();
    greenValue = colorSensor.green();
    blueValue = colorSensor.blue();
  }



  public void colorTelemetry() {
    //Putting the rgb values on the telemetry
      telemetry.addData("redValue is ", "%.2f", redValue);
      telemetry.addData("greenValue is ", "%.2f", greenValue);
      telemetry.addData("blueValue is ", "%.2f", blueValue);
      telemetry.update();
  }
}

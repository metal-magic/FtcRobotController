package org.firstinspires.ftc.teamcode.mmcenterstage.other;


import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import java.util.Date;
@TeleOp(name = "if your happy and you know it clap your hands")
public class ekansh_slay_teleop extends OpMode {
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;


    public Servo gripperServo1 = null;
    public Servo pivotServo = null;


    public Servo droneServo = null;


    public CRServo armMotor = null;


    public Date previousTime = new Date();


    //public float armSpeedCounter = 0;
    // TouchSensor touchSensor = null;


    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");


        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        pivotServo = hardwareMap.servo.get("pivotServo");


        droneServo = hardwareMap.servo.get("droneServo");


        // TouchSensor touchSensor = hardwareMap.touchSensor.get("touchSensor");


        armMotor = hardwareMap.crservo.get("armMotor");


        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);


        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        armMotor.setDirection(CRServo.Direction.REVERSE);


        ((ServoImplEx) pivotServo).setPwmRange(new PwmControl.PwmRange(500, 2500));




    }


    @Override
    public void loop() {
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");


        double y = -gamepad1.left_stick_y; // REVERSED
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        // Denominator is the largest motor power (abs value) or 1
        // This makes sure that the ratio stays the same
        // but only when at least one is out of range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        double motorSpeed;



        if (gamepad1.right_trigger >= 0.3F) {
            // Fine controls
            motorSpeed = 0.20;
        } else {
            // Reg speed
            motorSpeed = 0.75;
        }


        motorFrontLeft.setPower(frontLeftPower * motorSpeed);
        motorBackLeft.setPower(backLeftPower * motorSpeed);
        motorFrontRight.setPower(frontRightPower * motorSpeed);
        motorBackRight.setPower(backRightPower * motorSpeed);
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        boolean b = (distance <= 3);
        while (b){
            distance = distanceSensor.getDistance(DistanceUnit.INCH);
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            telemetry.addData("Distance (inches)", distance);
            telemetry.update();
        }
        // Print distance to the telemetry
        telemetry.addData("Distance (inches)", distance);
        telemetry.update();


        // Add a small delay to reduce loop frequency
        try {
            sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}

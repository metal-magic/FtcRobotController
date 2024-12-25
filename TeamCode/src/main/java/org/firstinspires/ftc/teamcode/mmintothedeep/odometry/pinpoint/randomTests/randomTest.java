package org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.randomTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.DriveToPoint;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name="randomTest", group="Pinpoint")
//@Disabled

public class randomTest extends LinearOpMode {

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5
    }

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,707,195,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 200, -658, AngleUnit.DEGREES, -138.13);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,353,428, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM,304, 1210, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM,260,1700, AngleUnit.DEGREES,132);



    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "motorBackRight");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(0, -180.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        boolean attarget = false;

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();


            if (gamepad1.y) {
                if (nav.driveTo(odo.getPosition(), TARGET_5, 0.4, 0)){
                    telemetry.addLine("at position #1!");
                }
            }


            if (!attarget) {
                //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
                leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            }

            telemetry.addData("current state:",stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();

        }
    }}
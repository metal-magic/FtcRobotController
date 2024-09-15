package org.firstinspires.ftc.teamcode.mmcenterstage.other;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Autonomous
public class DetectAprilTagNew extends LinearOpMode{

    @Override

    public void runOpMode() throws InterruptedException {
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //MyDriveTrain m = new MyDriveDrain();
        //m.rotate(90);


        //drawing information on the driver station camera screen
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(484.149, 484.149, 309.846, 272.681)
                .build();

        //VisionPortal myBestVisionPortal;
        //myBestVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "testWebcam"), tagProcessor);


        //stating the webcam
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .setCameraResolution(new Size(640, 480))
                .build();




        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            //rotate robot until it detects an AprilTag
            if ((tagProcessor.getDetections().size() != 0)&&((tagProcessor.getDetections().get(0).id % 2== 1))) {
                if (tagProcessor.getDetections().get(0).ftcPose.z <= 50) {
                    rotateRobot(0);
                } else {
                    rotateRobot(0.2);
                }
            } else {
                rotateRobot(0.2);
            }


            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                //sending telemetry values to the driver station
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("id", tag.id);

            }
            telemetry.update();



        }

    }

    private void rotateRobot(double value) {
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setPower(value);
        motorBackLeft.setPower(value);
        motorFrontRight.setPower(-1 * value);
        motorBackRight.setPower(-1 * value);
    }

    public void alignX(double minX, double maxX, int myTagID) {

        //drawing information on the driver station camera screen
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(484.149, 484.149, 309.846, 272.681)
                .build();


        //stating the webcam
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .setCameraResolution(new Size(640, 480))
                .build();


        int theIndex = 6;
        for (int i = 0; i <= 5; i++) {
            if (tagProcessor.getDetections().get(i).id == myTagID) {
                theIndex = myTagID;
                break;
            }
        }

        DriveTrainClass dtc = new DriveTrainClass();

        while (tagProcessor.getDetections().get(theIndex).ftcPose.x < (-0.5-minX)) { //0.5 is buffer
            dtc.strafe(-0.5, 0.5);
        }
        while (tagProcessor.getDetections().get(theIndex).ftcPose.x > (0.5 + maxX)) { //0.5 is buffer
            dtc.strafe(-0.5, 0.5);
        }
    }

    public void alignZ(double minZ, double maxZ, int myTagID) {

        //drawing information on the driver station camera screen
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(484.149, 484.149, 309.846, 272.681)
                .build();


        //stating the webcam
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .setCameraResolution(new Size(640, 480))
                .build();


        int theIndex = 6;
        for (int i = 0; i <= 5; i++) {
            if (tagProcessor.getDetections().get(i).id == myTagID) {
                theIndex = myTagID;
                break;
            }
        }

        DriveTrainClass dtc = new DriveTrainClass();

        while (tagProcessor.getDetections().get(theIndex).ftcPose.z < (-0.5-minZ)) { //0.5 is buffer
            dtc.moveStraightLine(-0.5, 0.5);
        }
        while (tagProcessor.getDetections().get(theIndex).ftcPose.z > (0.5 + maxZ)) { //0.5 is buffer
            dtc.moveStraightLine(-0.5, 0.5);
        }
    }

}
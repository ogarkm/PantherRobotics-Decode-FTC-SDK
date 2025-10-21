package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.scrimHWMap;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name="Omni Drive To AprilTag", group = "Concept")
public class RobotAprilTagOmni extends LinearOpMode
{
    // AprilTag Navigation Constants
    final double DESIRED_DISTANCE = 12.0; // inches
    final double SPEED_GAIN       = 0.02;
    final double STRAFE_GAIN      = 0.015;
    final double TURN_GAIN        = 0.01;

    final double MAX_AUTO_SPEED   = 0.5;
    final double MAX_AUTO_STRAFE  = 0.5;
    final double MAX_AUTO_TURN    = 0.3;

    // Robot hardware
    scrimHWMap robot = new scrimHWMap();

    // Vision Constants & Variables
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 20; // -1 for ANY tag
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    @Override public void runOpMode()
    {
        boolean targetFound = false;
        double  drive       = 0;
        double  strafe      = 0;
        double  turn        = 0;

        // Initialize vision and robot hardware
        initAprilTag();
        robot.init(hardwareMap);

        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // reduce motion blur
        }

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            // ---------- APRILTAG DETECTION ----------
            targetFound = false;
            desiredTag  = null;

            double minDistance = 1000;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    double distance = detection.ftcPose.range;
                    if (distance < minDistance) {
                        desiredTag = detection;
                        minDistance = distance;
                        targetFound = true;
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // ---------- DRIVE LOGIC ----------
            if (targetFound) {
                // auto drive
                double rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError     = desiredTag.ftcPose.yaw;

                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            } else {
                // manual drive
                drive  = -gamepad1.left_stick_y  / 2.0;
                strafe = -gamepad1.left_stick_x  / 2.0;
                turn   = -gamepad1.right_stick_x / 3.0;
            }

            // ---------- TELEMETRY ----------
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply motion
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }
    public void moveRobot(double x, double y, double yaw) {
        // Mecanum wheel mixing
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // scale to [-1,1]
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Set motor powers, reverse all to account for scrimHWMap directions
        robot.frontLeft.setPower(-frontLeftPower);
        robot.frontRight.setPower(-frontRightPower);
        robot.backLeft.setPower(-backLeftPower);
        robot.backRight.setPower(-backRightPower);
    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2); // higher decimation = faster processing, lower range

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    // Manually set camera gain and exposure (for webcams only)
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        // wait for camera to be streaming
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
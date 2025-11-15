// #---------------------------#
//  Basic Mecanum TeleOP
// 
//  Adapted from Game Manual 0
//  https://gm0.org
//  
// #---------------------------#

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


import org.firstinspires.ftc.teamcode.HwMap.Constants;
import org.firstinspires.ftc.teamcode.HwMap.hwMap;

import java.util.List;

@TeleOp(name="Mecanum_Strafe_Test", group="TeleOp")
public class TeleOP extends LinearOpMode {

    hwMap hw;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private static final boolean USE_WEBCAM = true;

    private double launchPower = 0.0;
    private double launchMode;


    @Override
    public void runOpMode() throws InterruptedException {
        hw = new hwMap(hardwareMap);
        initAprilTag();
        waitForStart();
        telemetry.addLine("Click A on TeleOP start to switch to unlinked mode");
        telemetry.update();

        launchPower = Constants.LaunchConstants.LAUNCH_MID;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            drive(x, y, rx);

            //intake
            if (gamepad1.right_trigger > 0) {
                intake(1);
            }
            else if (gamepad1.left_trigger > 0) {
                intake(-1);
            }
            else {intake(0);}

            //launch
            if (gamepad1.a && launchPower == 0.0) {
                launchPower = launchMode;
            } else if (gamepad1.a && launchPower != 0.0) {
                launchPower = 0.0;
            }
            launch(launchPower);

            //precision
            if (gamepad1.left_bumper) {
                drive(x, y, rx);
            } else if (gamepad1.right_bumper) {
                drive(x, y, rx);
            } else if (gamepad1.right_stick_button) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                AprilTagDetection targetTag = null;

                for (AprilTagDetection detection : detections) {
                    //
                    if (detection.metadata != null && detection.id == Constants.FieldConstants.redAprilTagID) {
                        targetTag = detection;
                        break;
                    }
                }
                if (targetTag != null) {
                    // INCHES
                    double xError = targetTag.ftcPose.x;  // left-right
                    double yError = targetTag.ftcPose.y;  // forward-back
                    double yawError = targetTag.ftcPose.yaw; // rotation error

                    double targetOffset = Constants.AutoAlignConstants.targetOffset;
                    double forwardError = yError - targetOffset;

                    double xPower = Range.clip(-xError * Constants.AutoAlignConstants.pX, -Constants.AutoAlignConstants.xCorrectionPower, Constants.AutoAlignConstants.xCorrectionPower);
                    double yPower = Range.clip(forwardError * Constants.AutoAlignConstants.pY, -Constants.AutoAlignConstants.yCorrectionPower, Constants.AutoAlignConstants.yCorrectionPower);
                    double rxPower = Range.clip(-yawError * Constants.AutoAlignConstants.pYaw, -Constants.AutoAlignConstants.yawCorrectionPower, Constants.AutoAlignConstants.yawCorrectionPower);

                    drive(xPower, yPower, rxPower);

                }
            }

            if (gamepad1.x) {launchMode = Constants.LaunchConstants.LAUNCH_MAX;}
            if (gamepad1.y) {launchMode = Constants.LaunchConstants.LAUNCH_MID;}
            if (gamepad1.b) {launchMode = Constants.LaunchConstants.LAUNCH_LOW;}

        }
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    private void drive(double x, double y, double rx) {
        double frontLeftPower = (y + x + rx) ;
        double backLeftPower = (y - x + rx) ;
        double frontRightPower = (y - x - rx);
        double backRightPower = (y + x - rx);

        hw.setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void intake(double power) {
        hw.setIntakePower(power);
    }

    private void launch(double power) {
        hw.setLauncherPower(power);
    }
}
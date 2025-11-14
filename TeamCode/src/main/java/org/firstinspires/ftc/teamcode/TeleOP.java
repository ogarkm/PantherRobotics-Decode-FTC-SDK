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

@TeleOp(name="Mecanum_Strafe_Test", group="TEST")
public class TeleOP extends LinearOpMode {

    hwMap hw;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private static final boolean USE_WEBCAM = true;


    // Mode = 1 = Linked Mode (Intake and Launcher)
    // Mode = 2 = UnLinked Mode

    public int mode = 1;

    private double launchPower;


    @Override
    public void runOpMode() throws InterruptedException {
        hw = new hwMap(hardwareMap);
        initAprilTag();
        waitForStart();
        telemetry.addLine("Click A on TeleOP start to switch to unlinked mode");
        telemetry.update();

        launchPower = Constants.LaunchConstants.LAUNCH_MAX;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            drive(x, y, rx, Constants.DriveConstants.NORMAL_SPEED_MULTIPLIER);

            //intake
            if (gamepad1.right_trigger > 0) {
                intake(1);
            }
            else if (gamepad1.left_trigger > 0) {
                intake(-1);
            }
            else {intake(0);}
            //launch
            if (gamepad1.a) {
                launch(launchPower);
            }
            else {launch(0);}
            //precision
            if (gamepad1.left_bumper) {
                drive(x, y, rx, Constants.DriveConstants.PRECISION_SPEED_MULTIPLIER);
            } else if (gamepad1.right_bumper) {
                drive(x, y, rx, Constants.DriveConstants.TURBO_SPEED_MULTIPLIER);
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

                    drive(xPower, yPower, rxPower, Constants.DriveConstants.NORMAL_SPEED_MULTIPLIER);

                }
            }

            if (gamepad1.x) {launchPower = Constants.LaunchConstants.LAUNCH_MAX;}
            if (gamepad1.y) {launchPower = Constants.LaunchConstants.LAUNCH_MID;}
            if (gamepad1.b) {launchPower = Constants.LaunchConstants.LAUNCH_LOW;}

            if (gamepad1.start) { if (mode == 1) {mode = 2;} else {mode = 1;} } //mode switcher

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

    private void drive(double x, double y, double rx, double mult) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1) * mult;
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        hw.setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void intake(double power) {
        hw.setIntakePower(power);
    }

    private void launch(double power) {
        hw.setLauncherPower(power);
    }
}
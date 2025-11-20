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

    // --- Simple rotation-only tracking parameters (tweak these on robot) ---
    private static final double P_YAW_ROTATE = 0.9;    // proportional gain (tweak)
    private static final double YAW_DEADBAND = 1.5;   // degrees - treat as on-target
    private static final double MAX_ROTATE_POWER = 0.5; // max rotate speed

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

            // Default: driver control
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

            //precision & auto-turn (HOLD right stick button to auto-rotate to tag)
            if (gamepad1.left_bumper) {
                drive(x, y, rx); // keep regular precision behavior (unchanged)
            } else if (gamepad1.right_bumper) {
                drive(x, y, rx); // unchanged
            } else if (gamepad1.right_stick_button) {
                // -------------------------------
                // SIMPLE ROTATION-ONLY TRACKING
                // -------------------------------
                List<AprilTagDetection> detections = aprilTag.getDetections();
                AprilTagDetection targetTag = null;

                for (AprilTagDetection detection : detections) {
                    if (detection.metadata != null && detection.id == Constants.FieldConstants.redAprilTagID) {
                        targetTag = detection;
                        break;
                    }
                }

                if (targetTag != null) {
                    // raw yaw from pipeline (may be radians or degrees)
                    double rawYaw = targetTag.ftcPose.yaw;
                    double yawDeg = rawYaw;
                    // convert if it looks like radians
                    if (Math.abs(rawYaw) <= (2.0 * Math.PI + 1e-6)) {
                        yawDeg = Math.toDegrees(rawYaw);
                    }

                    // If within deadband, stop turning
                    if (Math.abs(yawDeg) <= YAW_DEADBAND) {
                        drive(0, 0, 0); // on-target
                        telemetry.addData("AutoRotate", "ON TARGET (%.2f deg)", yawDeg);
                    } else {
                        // proportional control: want to rotate in the shortest direction to reduce yaw
                        // Note: sign: positive yawDeg means target to the right, so negative rx (clockwise) may be required depending on your motor mapping.
                        double rxPower = -yawDeg * P_YAW_ROTATE / 100.0; // scale down (yaw in degrees → power)
                        rxPower = Range.clip(rxPower, -MAX_ROTATE_POWER, MAX_ROTATE_POWER);
                        drive(0, 0, rxPower);
                        telemetry.addData("AutoRotate", "Turning (yaw=%.2f°, rx=%.3f)", yawDeg, rxPower);
                    }
                } else {
                    // no tag visible — hold rotation (stop rotating)
                    drive(0, 0, 0);
                    telemetry.addData("AutoRotate", "No Tag");
                }
            }

            if (gamepad1.x) {launchMode = Constants.LaunchConstants.LAUNCH_MAX;}
            if (gamepad1.y) {launchMode = Constants.LaunchConstants.LAUNCH_MID;}
            if (gamepad1.b) {launchMode = Constants.LaunchConstants.LAUNCH_LOW;}

            telemetry.update();
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


// #---------------------------#
//  Basic TeleOP with Turret Control (Should be Automatic tho)
//
//  FTC Team 12926 - Panther Robotics
//  Aarush Karnik
//
//  Adapted TeleOP from Game Manual 0
//  https://gm0.org
//  IDK if this actually works
// #---------------------------#

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Turret_Test", group="TeleOp")
public class TurretTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final double MANUAL_TURRET_SPEED_DEG = 1.0; // Degrees per loop
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_GEAR_RATIO = 3.5;
    private static final double TURRET_HOME_ANGLE = 0.0; // Home position in degrees

    // Proportional gain for turret correction.
    // Higher values will cause the turret to react faster, but may overshoot.
    final double TURN_GAIN = 0.02; // Adjusted gain for degree-based control

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        Servo turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        Servo turretRight = null;

        initAprilTag(); // April Tag Init

        try {
            turretRight = hardwareMap.get(Servo.class, "turretRight");
        } catch (Exception e) {
            telemetry.addLine("Single servo mode (turretRight not found)");
        }
        telemetry.addLine("Awaiting User start... ");
        telemetry.update();

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        double turretAngle = TURRET_HOME_ANGLE;
        int mode = 0;
        int team_id = 0;
        int team = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Robot Driving Logic (common to both modes)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            while (opModeIsActive() && team_id == 0) {
                telemetry.addLine("Press X for Blue team, Y for Red team");
                telemetry.update();
                if (gamepad1.x) {
                    team_id = 20; // Blue Alliance Tag ID
                    team = 1;

                } else if (gamepad1.y) {
                    team_id = 24; // Red Alliance Tag ID
                    team = 2;
                }
            }
            while (opModeIsActive() && mode == 0) { // User chooses mode
                telemetry.addLine("Press A for manual, B for Auto Turret (with manual drive)");
                telemetry.update();
                if (gamepad1.a) {
                    mode = 1;
                } else if (gamepad1.b) {
                    mode = 2;
                }
            }

            // Turret Control Logic
            if (mode == 1) { // Manual Mode
                double turretInput = 0.0;

                if (gamepad1.left_bumper) {
                    turretInput = -1.0;
                } else if (gamepad1.right_bumper) {
                    turretInput = 1.0;
                }

                turretAngle += turretInput * MANUAL_TURRET_SPEED_DEG;

                if (gamepad1.a) {
                    turretAngle = TURRET_HOME_ANGLE;
                }
            } else { // Auto Turret
                AprilTagDetection targetTag = null;
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null && detection.id == team_id) {
                        targetTag = detection;
                        break;
                    }
                }

                if (targetTag != null) {
                    // Yaw error states how far from center the tag is
                    double yaw = targetTag.ftcPose.yaw;

                    // A positive yaw error = target is to the left
                    turretAngle -= (yaw * TURN_GAIN);

                    telemetry.addData("Target Found", "ID: %d", targetTag.id);
                    telemetry.addData("Yaw Error", "%.2f degrees", yaw);
                } else {
                    telemetry.addLine("Target not visible.");
                }
            }

            double wrappedAngle = (turretAngle % 360 + 360) % 360;

            double normalizedTurretPos = wrappedAngle / 360.0;
            double servoPos = (normalizedTurretPos / SERVO_GEAR_RATIO);

            servoPos = (servoPos % 1.0 + 1.0) % 1.0;
            servoPos = Range.clip(servoPos, SERVO_MIN, SERVO_MAX);

            turretLeft.setPosition(servoPos);
            if (turretRight != null) {
                turretRight.setPosition(1.0 + servoPos);
            }

            telemetry.addData("Mode", mode == 1 ? "Manual" : "Auto");
            telemetry.addData("Turret Angle (Logical)", "%.2f", turretAngle);
            telemetry.addData("Turret Angle (Wrapped)", "%.2f", wrappedAngle);
            telemetry.addData("Servo Position", "%.3f", servoPos);
            telemetry.update();
        }
    }

    private void initAprilTag() {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }
}
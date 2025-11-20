package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
  Note: image (for reference): file:///mnt/data/cd91aa82-78b2-4889-bd3b-5570186a905a.png
*/

@TeleOp(name="Turret Subsystem TeleOP (PID Auto)", group="TeleOp")
public class TurretTest extends LinearOpMode {

    // Vision and AprilTag
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Turret Constants
    private static final double MANUAL_TURRET_SPEED_DEG = 1.0;
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_TO_TURRET_GEAR_RATIO = 3.5;
    private static final double TURRET_HOME_ANGLE = 0.0; // Home pos in deg (forward)

    // Old simple gain (kept for reference), but Auto now uses PID below
    final double TURN_GAIN = 0.02;

    // State machine modes
    private static final int STATE_SELECT_TEAM = 0;
    private static final int STATE_SELECT_MODE = 1;
    private static final int STATE_MANUAL_RUN = 2;
    private static final int STATE_AUTO_RUN = 3;

    // -------------------- PID / Auto Aim Parameters --------------------
    // Starting PID values (conservative). Tune on the robot for best behavior.
    private double kP = 0.02;   // proportional term (roughly similar to previous TURN_GAIN)
    private double kI = 0.00005;
    private double kD = 0.0015;

    // runtime PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime targetLostTimer = new ElapsedTime();

    // Deadband and power-to-angle conversion / limits
    private static final double POSITION_TOLERANCE = 1.5; // degrees tolerance to consider "on target"
    private static final double MIN_OUTPUT = 0.02;       // minimum PID output (to overcome stiction)
    private static final double MAX_OUTPUT = 4.0;        // max PID output (interpreted as degrees per loop)
    private static final double TARGET_LOST_TIMEOUT = 0.5; // seconds before we commit to return home

    // track last servo position to avoid wrap jumps
    private double lastServoPos = 0.5;

    @Override
    public void runOpMode() {
        // --- Hardware Initialization ---
        DcMotor frontLeftMotor = null;
        DcMotor backLeftMotor = null;
        DcMotor frontRightMotor = null;
        DcMotor backRightMotor = null;
        Servo turretLeft = null;
        Servo turretRight = null;

        try {
            frontLeftMotor = hardwareMap.dcMotor.get("fl");
            backLeftMotor = hardwareMap.dcMotor.get("bl");
            frontRightMotor = hardwareMap.dcMotor.get("fr");
            backRightMotor = hardwareMap.dcMotor.get("br");
            turretLeft = hardwareMap.get(Servo.class, "turretLeft");

            // Attempt to get the second servo, but don't fail if it's not there.
            try {
                turretRight = hardwareMap.get(Servo.class, "turretRight");
            } catch (Exception e) {
                telemetry.addLine("Single servo mode (turretRight not found)");
            }

        } catch (Exception e) {
            telemetry.addLine("CRITICAL ERROR: A required hardware device is missing.");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            // Terminate the OpMode gracefully if essential hardware is missing.
            sleep(5000);
            return;
        }

        // --- AprilTag Initialization ---
        initAprilTag();

        // --- Motor Configuration ---
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // --- Variable Initialization ---
        double turretAngle = TURRET_HOME_ANGLE;
        int currentState = STATE_SELECT_TEAM;
        int targetTagId = -1;
        boolean targetVisibleLastLoop = false;
        pidTimer.reset();
        targetLostTimer.reset();

        telemetry.addLine("Initialization Complete. Awaiting Start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Robot Driving Logic (runs in all states)
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


            // --- State Machine :) ---
            // TODO: Needs to be hardcoded to TELEOP for COMP

            // State 0: Choose Alliance
            if (currentState == STATE_SELECT_TEAM) {
                telemetry.addLine("Press X for Blue team, Y for Red team");
                if (gamepad1.x) {
                    targetTagId = 20; // Example Blue Alliance Tag ID
                    currentState = STATE_SELECT_MODE;
                } else if (gamepad1.y) {
                    targetTagId = 24; // Example Red Alliance Tag ID
                    currentState = STATE_SELECT_MODE;
                }
            }
            // State 1: Choose Turret Mode
            else if (currentState == STATE_SELECT_MODE) {
                telemetry.addLine("Press A for Manual Turret, B for Auto Turret");
                if (gamepad1.a) {
                    currentState = STATE_MANUAL_RUN;
                } else if (gamepad1.b) {
                    currentState = STATE_AUTO_RUN;
                }
            }
            // State 2: Manual Turret Control
            else if (currentState == STATE_MANUAL_RUN) {
                telemetry.addData("Mode", "Manual");
                double turretInput = 0.0;
                if (gamepad1.left_bumper) {
                    turretInput = -1.0;
                } else if (gamepad1.right_bumper) {
                    turretInput = 1.0;
                }
                turretAngle += turretInput * MANUAL_TURRET_SPEED_DEG;
            }

            // State 3: Automatic Turret Control (PID-based)
            else if (currentState == STATE_AUTO_RUN) {
                telemetry.addData("Mode", "Auto");
                AprilTagDetection targetTag = null;
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null && detection.id == targetTagId) {
                        targetTag = detection;
                        break; // Found the target, stop searching
                    }
                }

                double dt = pidTimer.seconds();
                pidTimer.reset();
                if (dt < 1e-3) dt = 1e-3;      // defensive
                if (dt > 1.0) dt = 1.0;       // cap in case loop stalled

                if (targetTag != null) {
                    // Reset lost-target timer
                    targetLostTimer.reset();

                    // targetTag.ftcPose.yaw units may be radians or degrees depending on pipeline.
                    // Heuristic: if magnitude <= 2*pi, assume radians and convert to degrees.
                    double rawYaw = targetTag.ftcPose.yaw;
                    double yawDeg = rawYaw;
                    if (Math.abs(rawYaw) <= (2.0 * Math.PI + 1e-6)) {
                        yawDeg = Math.toDegrees(rawYaw);
                    }
                    // yawDeg is the horizontal offset in degrees. Goal: bring yawDeg -> 0.
                    double error = yawDeg; // error in degrees (positive if target to the right)

                    // PID calculations (on degrees)
                    integral += error * dt;
                    // simple anti-windup clamp
                    integral = Range.clip(integral, -100.0, 100.0);
                    double derivative = (error - lastError) / dt;

                    double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);

                    // Deadband: if within tolerance, zero output and clear integral
                    if (Math.abs(error) < POSITION_TOLERANCE) {
                        pidOutput = 0.0;
                        integral = 0.0;
                    } else {
                        // enforce minimum effective output to overcome friction (preserve sign)
                        if (Math.abs(pidOutput) < MIN_OUTPUT) {
                            pidOutput = MIN_OUTPUT * Math.signum(pidOutput);
                        }
                    }

                    // Clamp pidOutput to a reasonable per-loop degree delta (so we move smoothly)
                    // Here pidOutput is treated as *degrees* to APPLY to turretAngle per loop.
                    double maxDegPerLoop = MAX_OUTPUT; // degrees per loop iteration (conservative)
                    double appliedDeg = Range.clip(pidOutput, -maxDegPerLoop, maxDegPerLoop);

                    // Update turret logical angle (note sign convention: target to right -> positive yaw -> we want to +angle to aim right)
                    // In prior code you used turretAngle -= yawError * TURN_GAIN; here we follow "positive yaw -> rotate turret positive"
                    turretAngle += appliedDeg;

                    lastError = error;

                    // Telemetry
                    telemetry.addData("Target Found", "ID: %d", targetTag.id);
                    telemetry.addData("Raw yaw", "%.4f", rawYaw);
                    telemetry.addData("Yaw (deg)", "%.2f", yawDeg);
                    telemetry.addData("Error (deg)", "%.2f", error);
                    telemetry.addData("PID out (deg)", "%.3f", pidOutput);
                    telemetry.addData("AppliedDeg", "%.3f", appliedDeg);
                } else {
                    // No target found in detections
                    // If we recently lost the target, let the turret coast (do nothing).
                    // If it's been missing longer than TARGET_LOST_TIMEOUT, return to home.
                    if (targetLostTimer.seconds() < TARGET_LOST_TIMEOUT) {
                        telemetry.addData("Status", "TRACKING LOST - Coasting");
                        telemetry.addData("Time Lost", "%.2f s", targetLostTimer.seconds());
                        // do not modify turretAngle (coast)
                    } else {
                        telemetry.addData("Status", "Target not visible. Returning to home.");
                        // gently interpolate back to home to avoid jumps
                        double wrapped = (turretAngle % 360 + 360) % 360;
                        double diffToHome = TURRET_HOME_ANGLE - wrapped;
                        // pick shortest path around circular domain
                        if (diffToHome > 180) diffToHome -= 360;
                        if (diffToHome < -180) diffToHome += 360;
                        // move a small step towards home
                        double homeStep = 3.0; // degrees per loop (conservative)
                        if (Math.abs(diffToHome) < homeStep) {
                            turretAngle = TURRET_HOME_ANGLE;
                        } else {
                            turretAngle += Math.signum(diffToHome) * homeStep;
                        }
                        // reset PID so integral doesn't carry over once new target appears
                        integral = 0;
                        lastError = 0;
                    }
                }
            }

            if ((currentState == STATE_MANUAL_RUN || currentState == STATE_AUTO_RUN) && gamepad1.start) {
                turretAngle = TURRET_HOME_ANGLE;
            }


            double wrappedAngle = (turretAngle % 360 + 360) % 360;

            double servoPos = getServoPosSafe(wrappedAngle);

            turretLeft.setPosition(servoPos);
            if (turretRight != null) {
                turretRight.setPosition(servoPos);
            }


            // --- Telemetry ---
            telemetry.addData("Turret Angle (Logical)", "%.2f", turretAngle);
            telemetry.addData("Turret Angle (Wrapped)", "%.2f", wrappedAngle);
            telemetry.addData("Servo Position", "%.3f", servoPos);
            telemetry.update();
        }
    }
    private double getServoPosSafe(double wrappedAngleDegrees) {
        // normalized turret rotation 0..1
        double turretRot = wrappedAngleDegrees / 360.0; // in [0,1)
        double servoRotations = turretRot * SERVO_TO_TURRET_GEAR_RATIO; // could be >1

        // fractional part in [0,1)
        double frac = servoRotations - Math.floor(servoRotations);

        // Choose the equivalent value (frac + n) that is nearest lastServoPos to avoid large jumps.
        double best = frac;
        double minDiff = Math.abs(best - lastServoPos);

        // consider neighbors ±1 (in case lastServoPos near edges)
        for (int k = -1; k <= 1; k++) {
            double candidate = frac + k;
            double diff = Math.abs(candidate - lastServoPos);
            if (diff < minDiff) {
                minDiff = diff;
                best = candidate;
            }
        }

        // ensure final is in 0..1 for servo.setPosition()
        double servoPos = Range.clip(best, SERVO_MIN, SERVO_MAX);
        lastServoPos = servoPos;
        return servoPos;
    }

    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }
}

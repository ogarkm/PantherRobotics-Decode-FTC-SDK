package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "BLUE - Net 12 Artifact Auto", group = "DECODE")
public class BlueNet12Auto extends LinearOpMode {

    // ----------------------------
    // PLACEHOLDERS / TUNABLES
    // ----------------------------
    // Road Runner starting pose (example - same as your previous)
    private static final Pose2d START_POSE = new Pose2d(48, -48, Math.toRadians(180));

    // AprilTag IDs (confirm your tags)
    private static final int BLUE_TAG_ID = 20;
    private static final int RED_TAG_ID = 24;

    // Odometry / pod placeholders (fill with your real values later)
    // These are intentionally placeholders. Replace with measured values.
    private static final double ODOM_LEFT_DISTANCE = 13.0;    // inches between left & right odometry wheels (example)
    private static final double ODOM_PERP_OFFSET = -2.0;     // inches from robot center to perp odometry wheel
    private static final double ODOM_WHEEL_DIAMETER = 1.26;  // inches (32mm = 1.259in)
    private static final int    ODOM_TICKS_PER_REV = 8192;   // example for high-res encoder

    // Drive tuning placeholders
    private static final double DRIVE_SPEED = 0.6;
    private static final double INTAKE_POWER = 1.0;

    // turret & shooter placeholders
    private Turret turret;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // Shared detection reference used by turret background thread
    private final AtomicReference<AprilTagDetection> lastDetection = new AtomicReference<>(null);

    // Thread handle for turret updates
    private Thread turretThread;

    @Override
    public void runOpMode() throws InterruptedException {
        // ----- INIT DRIVE -----
        // MecanumDrive constructor should accept hardwareMap and starting pose.
        // Make sure your MecanumDrive implementation supports odometry. We pass START_POSE.
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

        // ----- INIT TURRET -----
        turret = new Turret(hardwareMap); // assumes Turret(HardwareMap) constructor (from previous subsystem)
        turret.setAlliance(1); // blue alliance

        // ----- INIT VISION -----
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTagProcessor
        );

        telemetry.addLine("Init complete - waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Start background turret updater (keeps turret locking while we run blocking actions)
        startTurretThread();

        // ----- Autonomous Plan (12 artifact strategy, skeletal) -----
        // The overall pattern you described:
        //   1) Start at NET
        //   2) Go out, shoot preloads
        //   3) Go pick up 3
        //   4) Shoot
        //   5) Go to bar (release)
        //   6) Repeat pickups/shoot cycles to reach total 12
        //
        // This implementation uses simple reusable motion blocks (moveTo, driveThroughPickups, shootCycle).
        // You WILL need to tune positions and possibly split complex paths into smaller trajectories.

        // put robot in known starting pose (ensure RR localization uses this)
        drive.setPoseEstimate(START_POSE);

        // PHASE 0: small wait to stabilize vision / turret
        sleep(300);
        telemetry.addLine("Starting autonomous run");
        telemetry.update();

        // PHASE 1: Move out slightly to a shooting position and spin up shooter
        moveToPose(drive, new Pose2d(36, -48, Math.toRadians(180)), DRIVE_SPEED); // example
        spinUpShooter(); // placeholder - your team implements
        sleep(400); // allow flywheel to spin up (tune)

        // Shoot preloads (assume 3)
        shootArtifacts(3);

        // We will iterate: pick 3, shoot, go to bar, pick 3, shoot... until 12 total.
        // For clarity break into 3 cycles collecting groups of 3
        for (int cycle = 0; cycle < 3; cycle++) {
            // Navigate to pickup group (example coordinates; replace with measured ones)
            // Top-right group -> example vector from your previous auton (modify)
            moveToPose(drive, new Pose2d(48, 24, Math.toRadians(180)), DRIVE_SPEED);

            // Drive through the balls to intake them (use your intake method)
            startIntake();
            driveLineTo(drive, new Vector2d(24, 24), DRIVE_SPEED * 0.9); // drive THROUGH the group
            sleep(250); // ensure balls settled in intake
            stopIntake();

            // Return to shooting zone
            moveToPose(drive, new Pose2d(56, -48, Math.toRadians(270)), DRIVE_SPEED);

            // Spin up shooter and shoot collected artifacts
            spinUpShooter();
            sleep(400);
            shootArtifacts(3);

            // Go to the "bar thing" to release balls (example pose)
            moveToPose(drive, new Pose2d(60, -20, Math.toRadians(270)), DRIVE_SPEED * 0.7); // adjust
            releaseOnBar(); // placeholder - open servo / gate etc
            sleep(300);
            closeRelease(); // placeholder
        }

        // Final park in observation zone (modify as needed)
        moveToPose(drive, new Pose2d(48, -56, Math.toRadians(270)), DRIVE_SPEED);

        telemetry.addLine("Auton complete");
        telemetry.update();

        // Stop turret background thread
        stopTurretThread();
    }

    // ---------------------------
    // Motion helpers (blocking)
    // ---------------------------
    private void moveToPose(MecanumDrive drive, Pose2d target, double speed) {
        // Build a strafeToLinearHeading or lineTo call similar to your existing code.
        // This blocks until the action completes.
        Actions.runBlocking(
                drive.actionBuilder(drive.getPoseEstimate())
                        .strafeToLinearHeading(new Vector2d(target.getX(), target.getY()), target.getHeading())
                        .build()
        );
        // after motion, allow a short turret settle time
        sleep(75);
    }

    private void driveLineTo(MecanumDrive drive, Vector2d targetVec, double speed) {
        Actions.runBlocking(
                drive.actionBuilder(drive.getPoseEstimate())
                        .lineTo(targetVec)
                        .build()
        );
        sleep(75);
    }

    // ---------------------------
    // Turret background thread
    // ---------------------------
    private void startTurretThread() {
        turretThread = new Thread(() -> {
            while (opModeIsActive()) {
                // get latest detections from the pipeline
                List<AprilTagDetection> dets = aprilTagProcessor.getDetections();
                AprilTagDetection best = null;
                if (dets != null) {
                    for (AprilTagDetection d : dets) {
                        if (d.metadata != null && (d.id == BLUE_TAG_ID || d.id == RED_TAG_ID)) {
                            best = d;
                            break;
                        }
                    }
                }
                lastDetection.set(best);
                // update turret with the best detection (or null)
                turret.update(best);
                // ~50 Hz update
                try { Thread.sleep(20); } catch (InterruptedException ignored) {}
            }
        });
        turretThread.setDaemon(true);
        turretThread.start();
    }

    private void stopTurretThread() {
        if (turretThread != null && turretThread.isAlive()) {
            turretThread.interrupt();
            try { turretThread.join(200); } catch (InterruptedException ignored) {}
        }
    }

    // ---------------------------
    // Shooter / Intake placeholders
    // ---------------------------

    // Tell the flywheel to spin up (your implementation)
    private void spinUpShooter() {
        // TODO: call your shooter.spinUp(power) method
        // e.g. hw.shooter.spinUp(Constants.Shooter.SPIN_POWER);
    }

    // Shoot <count> artifacts by flicking servo(s)
    private void shootArtifacts(int count) {
        for (int i = 0; i < count && opModeIsActive(); i++) {
            flickGate();   // flick servo to feed one ball
            sleep(220);    // wait for ball to clear (tune)
        }
    }

    // Example flick action - placeholder
    private void flickGate() {
        // TODO: toggle your flicker servo here
        // ex: hw.flicker.setPosition(FOREGROUND); sleep(80); hw.flicker.setPosition(HOME);
    }

    private void startIntake() {
        // TODO: call your intake on method
        // ex: hw.intake.on(INTAKE_POWER);
    }

    private void stopIntake() {
        // TODO: call your intake off method
        // ex: hw.intake.off();
    }

    private void releaseOnBar() {
        // TODO: open any release mechanism to let balls out
    }

    private void closeRelease() {
        // TODO: close the release mechanism
    }
}

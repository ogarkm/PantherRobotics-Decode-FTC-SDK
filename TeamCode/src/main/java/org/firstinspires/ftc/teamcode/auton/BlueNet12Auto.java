package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hware.hwMapExt;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="12-Ball Auto with Odometry", group="DECODE")
public class BlueNet12Auto extends LinearOpMode {

    private hwMapExt hw;
    private Turret turret;

    // Odometry constants (placeholder)
    private static final double TICKS_PER_REV = Constants.DriveConstants.TICKS_PER_REVOLUTION;
    private static final double WHEEL_DIAMETER = 0.032; // meters (32 mm) bc gobilda odometry
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private static final double GEAR_RATIO = 1.0; // wheel -> encoder ratio

    // Robot pose
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0; // radians

    private int lastParallelLeft = 0;
    private int lastParallelRight = 0;
    private int lastPerp = 0;

    // Ball positions (placeholder)
    private static final double[][] BALL_POSITIONS = {
            {48, -48}, {56, -48}, {48, 24}, {24, 24},
            {56, -48}, {48, -56}, {24, 12}, {36, -24},
            {50, 10}, {42, -30}, {48, -48}, {50, -20}
    };

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new hwMapExt(hardwareMap);
        hw.initAprilTag();

        turret = new Turret(hw);
        turret.setAlliance(1); // 1=Blue, 2=Red

        telemetry.addLine("12-Ball Autonomous Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Reset encoders for odometry
        hw.resetEncoders();
        lastParallelLeft = hw.frontLeftMotor.getCurrentPosition();
        lastParallelRight = hw.frontRightMotor.getCurrentPosition();
        lastPerp = hw.backLeftMotor.getCurrentPosition();
        Thread turretThread = new Thread(() -> {
            while (opModeIsActive()) {
                AprilTagDetection tag = hw.getAprilTagById(20); // Blue tag
                turret.update(tag);
                sleep(20); // 50 Hz update
            }
        });
        turretThread.start();

        // Loop through all positions (drive and perform actions)
        for (int i = 0; i < BALL_POSITIONS.length; i++) {
            double targetX = BALL_POSITIONS[i][0];
            double targetY = BALL_POSITIONS[i][1];

            // Drive to target using odometry
            driveToPosition(targetX, targetY);

            // Collect or shoot depending on step
            if (i % 2 == 0) {
                startIntake();
                sleep(500); // simulate driving through balls
                stopIntake();
            } else {
                shootBall();
            }
        }

        // Stop turret thread
        turretThread.interrupt();
        turret.stop();
        hw.stopMotors();

        telemetry.addLine("12-Ball Auto Complete!");
        telemetry.update();
    }

    // ========== Odometry-Based Drive ==========

    private void driveToPosition(double targetX, double targetY) {
        double distanceThreshold = 1.0; // inches
        while (opModeIsActive()) {
            updateOdometry();

            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double distance = Math.hypot(dx, dy);
            if (distance < distanceThreshold) break;

            double angleToTarget = Math.atan2(dy, dx);
            double forward = Math.cos(angleToTarget - robotHeading) * 0.5; // max 50% power
            double strafe = Math.sin(angleToTarget - robotHeading) * 0.5;

            // Basic mecanum drive formula
            double fl = forward + strafe;
            double bl = forward - strafe;
            double fr = forward - strafe;
            double br = forward + strafe;

            // Clip powers
            fl = clip(fl, -1, 1);
            fr = clip(fr, -1, 1);
            bl = clip(bl, -1, 1);
            br = clip(br, -1, 1);

            hw.setMotorPowers(fl, fr, bl, br);

            telemetry.addData("Target", "(%.1f, %.1f)", targetX, targetY);
            telemetry.addData("Pose", "(%.1f, %.1f)", robotX, robotY);
            telemetry.addData("Distance", "%.2f", distance);
            telemetry.update();
        }

        hw.stopMotors();
    }

    private void updateOdometry() {
        int currentLeft = hw.frontLeftMotor.getCurrentPosition();
        int currentRight = hw.frontRightMotor.getCurrentPosition();
        int currentPerp = hw.backLeftMotor.getCurrentPosition();

        int deltaLeft = currentLeft - lastParallelLeft;
        int deltaRight = currentRight - lastParallelRight;
        int deltaPerp = currentPerp - lastPerp;

        lastParallelLeft = currentLeft;
        lastParallelRight = currentRight;
        lastPerp = currentPerp;

        double deltaLeftInches = ticksToInches(deltaLeft);
        double deltaRightInches = ticksToInches(deltaRight);
        double deltaForward = (deltaLeftInches + deltaRightInches) / 2.0;
        double deltaHeading = (deltaRightInches - deltaLeftInches) / 12.0; // 12 in robot width placeholder

        double deltaStrafe = ticksToInches(deltaPerp);

        robotHeading += deltaHeading;
        robotX += deltaForward * Math.cos(robotHeading) - deltaStrafe * Math.sin(robotHeading);
        robotY += deltaForward * Math.sin(robotHeading) + deltaStrafe * Math.cos(robotHeading);
    }

    private double ticksToInches(int ticks) {
        return (ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE * GEAR_RATIO * 39.37; // meters -> inches
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ========== Placeholder Methods ==========

    private void startIntake() {
        telemetry.addLine("Intake ON");
        telemetry.update();
        // hw.intakeMotor.setPower(1);
    }

    private void stopIntake() {
        telemetry.addLine("Intake OFF");
        telemetry.update();
        // hw.intakeMotor.setPower(0);
    }

    private void shootBall() {
        telemetry.addLine("Shooting Ball");
        telemetry.update();
        // hw.shooterMotor.setPower(shooterSpeed);
        sleep(300);
    }
}

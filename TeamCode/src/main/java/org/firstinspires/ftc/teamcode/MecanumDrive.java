package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
public final class MecanumDrive {

    // ===== TUNING PARAMETERS =====
    public static class Params {
        // goBILDA 5203 Series (312 RPM) motor constants //whats the correct rpm btw
        public double ticksPerRev = 537.7; //sanabhi this is correct right
        public double wheelRadius = 1.88976; // inches (96mm mecanum wheels)
        public double gearRatio = 1.0; // 1:1 if direct drive u prolly need to change the ratio bc idk as manan
        public double trackWidth = 15.0; // inches between left/right wheels

        //TUNE THESE using Road Runner tuning OpModes
        public double kS = 0.0; // Static friction
        public double kV = 0.0; // Velocity
        public double kA = 0.0; // Acceleration

        // PID gains for trajectory following - TUNE THESE
        public double lateralMultiplier = 1.0;

        // IMU orientation
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacing =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacing =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    }

    public static Params PARAMS = new Params();

    // ===== HARDWARE =====
    private final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private final IMU imu;

    // Robot pose tracking
    public Pose2d pose;

    // ===== CONSTRUCTOR =====
    public MecanumDrive(HardwareMap hardwareMap, Pose2d startPose) {
        this.pose = startPose;

        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // Set motor directions - ADJUST IF NEEDED U KIDS
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set brake mode
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                PARAMS.logoFacing,
                PARAMS.usbFacing
        ));
        imu.initialize(parameters);
    }

    // ===== DRIVE CONTROL =====

    public void setDrivePowers(PoseVelocity2d powers) {
        // Mecanum drive kinematics
        double x = powers.linearVel.x;
        double y = powers.linearVel.y;
        double turn = powers.angVel;

        double frontLeftPower = y + x + turn;
        double backLeftPower = y - x + turn;
        double backRightPower = y + x - turn;
        double frontRightPower = y - x - turn;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(backRightPower), Math.abs(frontRightPower))
        ));

        leftFront.setPower(frontLeftPower / maxPower);
        leftBack.setPower(backLeftPower / maxPower);
        rightBack.setPower(backRightPower / maxPower);
        rightFront.setPower(frontRightPower / maxPower);
    }

    public void updatePoseEstimate() {
        // Simplified

        // you need to implement proper odometry using
        // Odometry pods
        // See Road Runner tuning docs i guess
    }

    // ===== TRAJECTORY BUILDER =====

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(this, beginPose);
    }

    public class TrajectoryActionBuilder {
        private final MecanumDrive drive;
        private Pose2d currentPose;
        private Pose2d targetPose;

        public TrajectoryActionBuilder(MecanumDrive drive, Pose2d beginPose) {
            this.drive = drive;
            this.currentPose = beginPose;
            this.targetPose = beginPose;
        }

        public TrajectoryActionBuilder strafeToLinearHeading(Vector2d position, double heading) {
            this.targetPose = new Pose2d(position, heading);
            return this;
        }

        public TrajectoryActionBuilder strafeTo(Vector2d position) {
            this.targetPose = new Pose2d(position, currentPose.heading.toDouble());
            return this;
        }

        public TrajectoryActionBuilder lineTo(Vector2d position) {
            double angle = Math.atan2(
                    position.y - currentPose.position.y,
                    position.x - currentPose.position.x
            );
            this.targetPose = new Pose2d(position, angle);
            return this;
        }

        public TrajectoryActionBuilder turn(double angle) {
            this.targetPose = new Pose2d(
                    currentPose.position,
                    currentPose.heading.toDouble() + angle
            );
            return this;
        }
        public Action build() {
            final Pose2d target = targetPose;
            final Pose2d start = currentPose;

            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                    }

                    // Calculate errors
                    double xError = target.position.x - drive.pose.position.x;
                    double yError = target.position.y - drive.pose.position.y;
                    double headingError = target.heading.toDouble() - drive.pose.heading.toDouble();

                    while (headingError > Math.PI) headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;

                    // Check
                    double distanceError = Math.sqrt(xError * xError + yError * yError);
                    if (distanceError < 0.5 && Math.abs(headingError) < Math.toRadians(2)) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        drive.pose = target;
                        return false; // Done
                    }

                    double kP = 0.05; // Tune this
                    double kPHeading = 0.5; // Tune this

                    double heading = drive.pose.heading.toDouble();
                    double xPowerRobot = xError * Math.cos(-heading) - yError * Math.sin(-heading);
                    double yPowerRobot = xError * Math.sin(-heading) + yError * Math.cos(-heading);

                    double xPower = xPowerRobot * kP;
                    double yPower = yPowerRobot * kP;
                    double turnPower = headingError * kPHeading;

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(xPower, yPower),
                            turnPower
                    ));

                    // Update pose (simplified - should use odometry - u got that sanabhi)
                    drive.pose = new Pose2d(
                            drive.pose.position.x + xPower * 0.02,
                            drive.pose.position.y + yPower * 0.02,
                            drive.pose.heading.toDouble() + turnPower * 0.02
                    );

                    // Telemetry for debugging
                    packet.put("x", drive.pose.position.x);
                    packet.put("y", drive.pose.position.y);
                    packet.put("heading", Math.toDegrees(drive.pose.heading.toDouble()));
                    packet.put("xError", xError);
                    packet.put("yError", yError);
                    packet.put("headingError", Math.toDegrees(headingError));

                    return true;
                }
            };
        }
    }
}

/*
 *  SANABHI CAN U CHECK IF THIS STUFF IS THERE I LOOKED AROUND AND APPARENTLY WE NEED THIS?
 *    implementation "com.acmerobotics.roadrunner:ftc:0.1.25"
 *    implementation "com.acmerobotics.roadrunner:core:1.0.1"
 *    implementation "com.acmerobotics.roadrunner:actions:1.0.1"
 *    implementation "com.acmerobotics.dashboard:dashboard:0.4.6"
 *
 *  I ASSUME FOR THE ROBOT CONFIG WE USE EXISTING ONES, IS IMU THERE!?!??
 *  we'll probably need to debug the motor movements
 *
 * this is a simplified code so you still need to implement odometry + tuning
 *
 *field center assumed to be 0,0 btw
 */
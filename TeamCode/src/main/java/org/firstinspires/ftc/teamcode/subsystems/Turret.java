package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hware.hwMapExt;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


public class Turret {

    private final hwMapExt hardware;
    private double turretPower = 1.0;
    private int alliance; // Used for Tracking

    private int[] apriltags = {20, 24};
    private AprilTagDetection apriltag;
    private double turretAngle = Constants.TurretConstants.TURRET_HOME_ANGLE;


    private double kP = 0.02;
    private double kI = 0.00005;
    private double kD = 0.0015;

    private double integral = 0;
    private double lastError = 0;
    private double lastServoPos = 0.5;

    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime lostTimer = new ElapsedTime();

    private static final double POSITION_TOLERANCE = 1.5;
    private static final double MIN_OUTPUT = 0.02;
    private static final double MAX_OUTPUT = 4.0;
    private static final double LOST_TIMEOUT = 0.5;

    public enum TurretState {
        LAUNCH,
        IDLE,
        EXTAKE,
        RESET
    }
    private TurretState currentState = TurretState.IDLE;

    public Turret(hwMapExt hardware) {
        this.hardware = hardware;
    }

    public void setAlliance(int alliance) {
        this.alliance = alliance;
    }

    public void stop() {
        if (currentState != TurretState.IDLE) {
            setTurretState(TurretState.IDLE);
        }

        hardware.turretOff();
    }

    public void launchTurret() {
        hardware.setTurretPower(turretPower);
    }

    public void setLaunchPower(double power) {
        this.turretPower = power;
    }

    private void resetTurret() {
        hardware.initAprilTag();

    }

    public void autoLockLogic() {

        apriltag = hardware.getAprilTagById(apriltags[alliance - 1]);

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt < 1e-3) dt = 1e-3;
        if (dt > 1.0) dt = 1.0;

        if (apriltag != null) {

            lostTimer.reset();

            double rawYaw = apriltag.ftcPose.yaw;
            double yawDeg = (Math.abs(rawYaw) <= (2 * Math.PI)) ?
                    Math.toDegrees(rawYaw) : rawYaw;

            double error = yawDeg;
            integral += error * dt;
            integral = Range.clip(integral, -100, 100);

            double derivative = (error - lastError) / dt;

            double output = (kP * error) + (kI * integral) + (kD * derivative);

            if (Math.abs(error) < POSITION_TOLERANCE) {
                output = 0;
                integral = 0;
            } else if (Math.abs(output) < MIN_OUTPUT) {
                output = MIN_OUTPUT * Math.signum(output);
            }

            double appliedDeg = Range.clip(output, -MAX_OUTPUT, MAX_OUTPUT);

            turretAngle += appliedDeg;
            lastError = error;
        }
        else {
            if (lostTimer.seconds() < LOST_TIMEOUT) {
            } else {
                double wrapped = (turretAngle % 360 + 360) % 360;
                double diff = Constants.TurretConstants.TURRET_HOME_ANGLE - wrapped;

                if (diff > 180) diff -= 360;
                if (diff < -180) diff += 360;

                double step = 3.0;
                if (Math.abs(diff) < step) {
                    turretAngle = Constants.TurretConstants.TURRET_HOME_ANGLE;
                } else {
                    turretAngle += Math.signum(diff) * step;
                }

                integral = 0;
                lastError = 0;
            }
        }

        double wrappedAngle = (turretAngle % 360 + 360) % 360;
        double servoPos = getServoPosSafe(wrappedAngle);
        hardware.setTurretPos(servoPos);
    }

    private double getServoPosSafe(double wrappedAngleDegrees) {

        double turretRot = wrappedAngleDegrees / 360.0;
        double servoRot = turretRot * Constants.TurretConstants.SERVO_TO_TURRET_GEAR_RATIO;

        double frac = servoRot - Math.floor(servoRot);

        double best = frac;
        double minDiff = Math.abs(frac - lastServoPos);

        for (int k = -1; k <= 1; k++) {
            double cand = frac + k;
            double diff = Math.abs(cand - lastServoPos);
            if (diff < minDiff) {
                minDiff = diff;
                best = cand;
            }
        }

        double servoPos = Range.clip(best,
                Constants.TurretConstants.SERVO_MIN,
                Constants.TurretConstants.SERVO_MAX);

        lastServoPos = servoPos;
        return servoPos;
    }

    public void setTurretState(TurretState state) {
        this.currentState = state;

        switch (state) {
            case LAUNCH:
                launchTurret();
                break;
            case IDLE:
                hardware.turretOff();
                break;
            case EXTAKE:
                hardware.setTurretPower(Constants.TurretConstants.EXTAKE_POWER);
                break;
            case RESET:
                resetTurret();
                hardware.turretOff();
                break;
        }
    }
    public TurretState getTurretState() {
        return currentState;
    }

}

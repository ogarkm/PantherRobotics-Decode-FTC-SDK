package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Hware.hwMapExt;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Turret {

    private static final int BLUE_TAG = 20;
    private static final int RED_TAG = 24;

    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;

    private static final double MAX_TURRET_DEG = 250;
    private static final double SERVO_RANGE_DEG = 300;

    // PID tuned for 10–20 FPS Logitech (idk)
    private static final double KP = 0.015;
    private static final double KI = 0.0001;
    private static final double KD = 0.0015;

    private static final double MAX_PID_OUTPUT = 5;   // deg per update
    private static final double DEADZONE = 1.2;       // ignore tiny errors

    private final ServoEx turretServo;

    private double turretAngleDeg = 0;
    private double integral = 0;
    private double lastError = 0;

    private int alliance = 1;
    private ElapsedTime loopTimer = new ElapsedTime();

    public Turret(hwMapExt hw) {
        turretServo = new ServoEx(hw, "turretServo");
        loopTimer.reset();
    }

    public void setAlliance(int a) {
        alliance = (a == 1 || a == 2) ? a : 1;
    }

    public void update(AprilTagDetection tag) {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) dt = 0.01;

        if (tag == null || !isCorrectTag(tag.id)) {
            holdPosition();
            return;
        }

        double yawDeg = tag.ftcPose.yaw;

        yawDeg = lowPass(yawDeg, 0.3);

        double error = yawDeg;

        if (Math.abs(error) < DEADZONE) error = 0;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = KP * error + KI * integral + KD * derivative;
        pid = clamp(pid, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

        turretAngleDeg += pid;
        if (Math.abs(turretAngleDeg) > MAX_TURRET_DEG) {
            if (turretAngleDeg > 0) turretAngleDeg -= 360;
            else turretAngleDeg += 360;
        }

        setServoToAngle(turretAngleDeg);
    }


    private void setServoToAngle(double angleDeg) {
        double normalized = angleDeg / SERVO_RANGE_DEG;
        while (normalized < 0) normalized += 1;
        while (normalized > 1) normalized -= 1;
        normalized = clamp(normalized, SERVO_MIN, SERVO_MAX);
        turretServo.setPosition(normalized);
    }

    private boolean isCorrectTag(int id) {
        return alliance == 1 ? id == BLUE_TAG : id == RED_TAG;
    }

    private void holdPosition() {
        setServoToAngle(turretAngleDeg); // freeze turret
    }

    private double lowPass(double input, double factor) {
        // Smoother yaw = less jitter
        return (factor * input) + (1 - factor) * lastError;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public static class ServoEx {
        private final com.qualcomm.robotcore.hardware.Servo servo;

        public ServoEx(hwMapExt hw, String id) {
            servo = hw.turretservo;
        }

        public void setPosition(double p) {
            servo.setPosition(p);
        }
    }
}

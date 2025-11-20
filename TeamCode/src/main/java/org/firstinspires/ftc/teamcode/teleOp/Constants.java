package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {

    // Drive Constants
    public static class DriveConstants {
        public static final String FRONT_LEFT_MOTOR = "fl";
        public static final String FRONT_RIGHT_MOTOR = "fr";
        public static final String BACK_LEFT_MOTOR = "bl";
        public static final String BACK_RIGHT_MOTOR = "br";


        public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static DcMotor.ZeroPowerBehavior MOTOR_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;

        public static DcMotor.RunMode MOTOR_RUNMODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

        public static final double TRACK_WIDTH = 13.5; // inches (distance between left and right wheels)
        public static final double WHEEL_BASE = 13.5; // inches (distance between front and back wheels)
        public static final double WHEEL_DIAMETER = 4.0; // inches
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double TICKS_PER_REVOLUTION = 537.6; // GoBILDA 5202 Yellow Jacket
        public static final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

        // Drive characteristics
        public static final double MAX_VELOCITY = 30.0; // inches per second
        public static final double MAX_ANGULAR_VELOCITY = Math.toRadians(180.0); // radians per second
        public static final double MAX_ACCELERATION = 30.0; // inches per second squared

        // PID coefficients for autonomous driving
        public static final double DRIVE_kP = 0.1;
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;

        public static final double TURN_kP = 0.1;
        public static final double TURN_kI = 0.0;
        public static final double TURN_kD = 0.0;

        public static final double NORMAL_SPEED_MULTIPLIER = 1.0;
        public static final double PRECISION_SPEED_MULTIPLIER = 0.4;
        public static final double TURBO_SPEED_MULTIPLIER = 1.0;
    }

    public static class TurretConstants {
        public static final double SERVO_MIN = 0.0;
        public static final double SERVO_MAX = 1.0;

        public static final double MANUAL_TURRET_SPEED_DEG = 1.0;

        public static final double SERVO_TO_TURRET_GEAR_RATIO = 3.5;
        public static final double TURRET_HOME_ANGLE = 0.0; // Home pos in deg (forward)

        public static final double TURN_GAIN = 0.02;

        public static final double EXTAKE_POWER = 0.3;


        public static final String LEFT_TURRET_SERVO = "turretservo";

        public static final String HOOD_TURRET_SERVO = "hoodservo";


        public static final String TURRET_RIGHT_MOTOR = "rturret";
        public static final String TURRET_LEFT_MOTOR = "lturret";

        public static final DcMotorSimple.Direction TURRET_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    }
}
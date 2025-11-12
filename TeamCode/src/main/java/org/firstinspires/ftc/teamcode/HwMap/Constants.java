package org.firstinspires.ftc.teamcode.HwMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {

    // Drive Constants
    public static class DriveConstants {
        public static final String FRONT_LEFT_MOTOR = "fl";
        public static final String FRONT_RIGHT_MOTOR = "fr";
        public static final String BACK_LEFT_MOTOR = "bl";
        public static final String BACK_RIGHT_MOTOR = "br";

        public static final String INTAKE = "intake";

        public static final String LEFT_LAUNCHER = "flauncher";
        public static final String RIGHT_LAUNCHER = "rlauncher";

        public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final DcMotorSimple.Direction INTAKE_DIR = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction R_LAUNCH_DIR = DcMotorSimple.Direction.REVERSE;

        public static final DcMotorSimple.Direction F_LAUNCH_DIR = DcMotorSimple.Direction.FORWARD;


        public static DcMotor.ZeroPowerBehavior MOTOR_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;

        public static DcMotor.RunMode MOTOR_RUNMODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

        public static DcMotor.RunMode AUTON_RUNMODE = DcMotor.RunMode.RUN_USING_ENCODER;

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

        public static final double LAUNCH_MAX = 1.0;
        public static final double LAUNCH_MID = 0.6;
        public static final double LAUNCH_LOW = 0.2;
    }

    // Controller Constants
//    public static class ControllerConstants {
//        // Controller deadzones
//        public static final double LEFT_STICK_DEADZONE = 0.1;
//        public static final double RIGHT_STICK_DEADZONE = 0.1;
//        public static final double TRIGGER_DEADZONE = 0.1;
//
//        // Button mappings - Gamepad1 (Driver)
//        public static final int PRECISION_MODE_BUTTON = gamepad1.right_trigger;
//        public static final int TURBO_MODE_BUTTON = gamepad1.left_trigger;
//        public static final int FIELD_CENTRIC_TOGGLE_BUTTON = gamepad1.x;
//        public static final int RESET_IMU_BUTTON = gamepad1.y;
//
//        // Button mappings - Gamepad2 (Operator)
//        public static final int INTAKE_BUTTON = gamepad2.a;
//        public static final int OUTTAKE_BUTTON = gamepad2.b;
//        public static final int LIFT_UP_BUTTON = gamepad2.dpad_up;
//        public static final int LIFT_DOWN_BUTTON = gamepad2.dpad_down;
//    }
}
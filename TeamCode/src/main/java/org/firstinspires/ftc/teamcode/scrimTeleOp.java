package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.scrimHWMap;

@TeleOp(name="Mecanum_TeleOp_Shooter", group="TeleOp")
public class scrimTeleOp extends LinearOpMode {

    scrimHWMap robot = new scrimHWMap();

    // Shooter control
    private boolean shooterOn = false;
    private double shooterTargetRPM = 2800.0; // default target RPM, tune for your flywheels
    private final double RPM_STEP = 50.0;

    // Power multipliers
    private double baseDriveMultiplier = 0.8;
    private final double PRECISION_MULT = 0.5;
    private final double TURBO_MULT = 1.2; // will be clamped to 1.0 max

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // If your shooter supports velocity control, you'll want to use setVelocity (ticks/sec or RPM conversion)
        // Example uses simple power control fallback. Prefer DcMotorEx.setVelocity with properly tuned PIDF.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---------- DRIVE ----------
            double driveY = -gamepad1.left_stick_y; // forward
            double driveX = gamepad1.left_stick_x;  // strafe
            double turn   = gamepad1.right_stick_x; // rotation

            // apply deadzone
            driveX = applyDeadzone(driveX, 0.05);
            driveY = applyDeadzone(driveY, 0.05);
            turn = applyDeadzone(turn, 0.05);

            double multiplier = baseDriveMultiplier;
            if (gamepad1.left_bumper) multiplier *= PRECISION_MULT;
            if (gamepad1.right_bumper) multiplier *= TURBO_MULT;
            multiplier = Range.clip(multiplier, 0.0, 0.5); //CHNAGE 0.5 to make max speed faster

            // mecanum wheel mixing (robot-centric)
            double fl = driveY + driveX + turn;
            double fr = driveY - driveX - turn;
            double bl = driveY - driveX + turn;
            double br = driveY + driveX - turn;

            // scale to [-1,1]
            double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            robot.frontLeft.setPower(fl * multiplier);
            robot.frontRight.setPower(fr * multiplier);
            robot.backLeft.setPower(bl * multiplier);
            robot.backRight.setPower(br * multiplier);

            // quick stop / zero rotation
            if (gamepad1.a) {
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);
            }

            // ---------- INTAKE ----------
            // left_trigger: intake in, left_bumper (on gamepad2) intake out
            double intakePower = 0.0;
            if (gamepad2.left_trigger > 0.1) {
                intakePower = Range.clip(gamepad2.left_trigger, 0.0, 1.0); // in
            } else if (gamepad2.left_bumper) {
                intakePower = -0.8; // out
            } else {
                intakePower = 0.0;
            }
            robot.intake.setPower(intakePower);

            // ---------- SHOOTER ----------
            // Right bumper toggles shooter on/off (preset)
            if (gamepad2.right_bumper && gamepad2.right_bumper != gamepad2.right_bumper /*no-op to suppress lint*/) {
                // noop (keeps lint quiet)
            }
            // We'll implement an edge-triggered toggle
            if (gamepad2.right_bumper && !gamepad2.right_bumper /* placeholder */) {
                // unreachable - placeholder for conceptual reading
            }

            // Simpler approach: use X / Y to set presets, B to stop, Dpad +/- to adjust RPM
            if (gamepad2.x) {
                shooterOn = true;
                shooterTargetRPM = 2000; // low preset
            }
            if (gamepad2.y) {
                shooterOn = true;
                shooterTargetRPM = 3000; // high preset
            }
            if (gamepad2.b) {
                shooterOn = false;
            }

            if (gamepad2.dpad_up) {
                shooterTargetRPM += RPM_STEP;
            } else if (gamepad2.dpad_down) {
                shooterTargetRPM -= RPM_STEP;
            }

            shooterTargetRPM = Range.clip(shooterTargetRPM, 0, 6000);

            // If your motors are DcMotorEx and support setVelocity (ticks per second), please convert RPM to ticks/sec.
            // For an example using REV motor with 28 ticks per revolution, ticksPerRev = 28 * gearRatio etc.
            // For portability, we set open-loop power proportional to target RPM as a fallback:
            double shooterPower = 0.0;
            if (shooterOn) {
                // map RPM to power (naive mapping — tune for your hardware)
                shooterPower = shooterTargetRPM / 4000.0; // 4000 rpm -> 1.0 power (example)
                shooterPower = Range.clip(shooterPower, 0.0, 1.0);
            } else {
                shooterPower = 0.0;
            }

            // If shooter motors are DcMotorEx and you want velocity control:
            if (robot.shooterLeft instanceof DcMotorEx && robot.shooterRight instanceof DcMotorEx) {
                DcMotorEx sL = robot.shooterLeft;
                DcMotorEx sR = robot.shooterRight;
                // Example: if you have calculated ticksPerMinute, setVelocity accepts ticksPerSecond depending on SDK version.
                // Here: fallback to setPower for compatibility. Replace with setVelocity(...) if you know your encoder ticks/rev.
                sL.setPower(shooterPower);
                sR.setPower(shooterPower);
            } else {
                robot.shooterLeft.setPower(shooterPower);
                robot.shooterRight.setPower(shooterPower);
            }

            // ---------- FEED / INDEXER ----------
            // Right trigger on gamepad2 acts as "feed" — in a physical robot you'd run an indexer servo or a feeder motor.
            if (gamepad2.right_trigger > 0.2) {
                // If you have indexer servo, activate it here to feed one ball at a time
                robot.intake.setPower(0.5);
            }

            // ---------- TELEMETRY ----------
            telemetry.addData("Drive multiplier", multiplier);
            telemetry.addData("Shooter on", shooterOn);
            telemetry.addData("Shooter target RPM", shooterTargetRPM);
            telemetry.addData("Shooter power (open-loop)", shooterPower);
            telemetry.addData("Intake power", intakePower);
            telemetry.update();

            sleep(20); // small loop delay
        }
    }

    private double applyDeadzone(double val, double dead) {
        return Math.abs(val) < dead ? 0.0 : val;
    }
}

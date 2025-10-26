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
//            robot.frontRight.setPower(fr * multiplier);
//            robot.backLeft.setPower(bl * multiplier);
//            robot.backRight.setPower(br * multiplier);



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
            telemetry.update();

            sleep(20); // smalscrimTeleOpl loop delay
        }
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Intake {

    private final DcMotorEx frontMotor, backMotor;
    private double runPower = 0.85;

    // Initialize intake motors
    public Intake(HardwareMap hw) {
        // Change these names to match your config
        frontMotor = hw.get(DcMotorEx.class, "frontIntake");
        backMotor  = hw.get(DcMotorEx.class, "backIntake");

        // Motor directions
        frontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake when stopped
        frontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run without encoders
        frontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Intake inward (both)
    public void in() {
        setBoth(runPower);
    }

    // Intake outward (both)
    public void out() {
        setBoth(-runPower);
    }

    // Stop both
    public void stop() {
        setBoth(0.0);
    }

    // Set base intake power
    public void setPower(double power) {
        runPower = Range.clip(power, 0.0, 1.0);
    }

    // Reverse both briefly to clear jams
    public void reverseFor(double durationSeconds, double power) {
        power = Range.clip(power, 0.0, 1.0);
        ElapsedTime t = new ElapsedTime();
        setBoth(-power);
        while (t.seconds() < durationSeconds && !Thread.currentThread().isInterrupted()) {}
        stop();
    }

    // Optional: closed-loop velocity control for both
    public void setVelocityRPM(double rpm, double ticksPerRev) {
        double cps = rpm * ticksPerRev / 60.0;

        frontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontMotor.setVelocity(cps);
        backMotor.setVelocity(cps);
    }

    // Internal: set power on both motors
    private void setBoth(double p) {
        frontMotor.setPower(p);
        backMotor.setPower(p);
    }

    // Optional helpers if you ever want to move them separately

    public void frontOnly(double p) {
        frontMotor.setPower(p);
    }

    public void backOnly(double p) {
        backMotor.setPower(p);
    }
}

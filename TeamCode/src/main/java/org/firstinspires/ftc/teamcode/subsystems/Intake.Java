package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Hware.hwMapExt;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.teleOp.Constants;

public class Intake {
    private double runPower = 1;
    private final hwMapExt hardware;

    // Initialize intake motors

    public enum IntakeState {
        INTAKE,
        EXTAKE,
        IDLE,
        STOP
    }
    private Intake.IntakeState currentState = Intake.IntakeState.IDLE;
    public Intake(hwMapExt hardware) {
        this.hardware = hardware;
    }

    public void setIntakeState(Intake.IntakeState state) {
        this.currentState = state;

        switch (state) {
            case INTAKE:
                hardware.frontIntakeMotor.setPower(Constants.IntakeConstants.INTAKE_POWER);
                hardware.backIntakeMotor.setPower(Constants.IntakeConstants.INTAKE_POWER);
                break;
            case EXTAKE:
                hardware.frontIntakeMotor.setPower(Constants.IntakeConstants.EXTAKE_POWER);
                hardware.backIntakeMotor.setPower(Constants.IntakeConstants.EXTAKE_POWER);
                break;
            case IDLE:
                hardware.frontIntakeMotor.setPower(Constants.IntakeConstants.IDLE_POWER);
                hardware.backIntakeMotor.setPower(Constants.IntakeConstants.IDLE_POWER);
                break;
            case STOP:
                break;
        }
    }
}

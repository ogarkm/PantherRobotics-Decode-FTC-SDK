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

public class Lift {
    private double runPower = 1;
    private final hwMapExt hardware;

    // Initialize intake motors

    public enum LiftState {
        LIFT_UP,
        LIFT_DOWN,
        LOCK,
        IDLE
    }
    private Lift.LiftState currentState = Lift.LiftState.IDLE;
    public Lift(hwMapExt hardware) {
        this.hardware = hardware;
    }

    public void setLiftState(Lift.LiftState state) {
        this.currentState = state;

        switch (state) {
            case LIFT_UP:
                hardware.setMotorTargetPositions(18);
                hardware.setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.setMotorTargetPositions(18);
                hardware.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.setMotorPowers(1,1,1,1);
                break;
            case LIFT_DOWN:
                hardware.setMotorPowers(0,0,0,0);
                break;
            case LOCK:
                hardware.setServoPosition(Constants.LiftConstants.PTO_LOCK);
                setLiftState(LiftState.LIFT_UP);
                break;
            case IDLE:
                hardware.setServoPosition(Constants.LiftConstants.PTO_UNLOCK);
                break;
        }
    }
}

package org.firstinspires.ftc.teamcode.teleOp;

import org.firstinspires.ftc.teamcode.Hware.hwMapExt;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GameState;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class StateMachine {
    private RobotState currentRobotState = RobotState.INIT;
    private GameState currentGameState = GameState.IDLE;
    private final DriveTrain m_driveTrain;
    private final hwMapExt hardware;
    private final Intake m_intake; 


    public StateMachine(hwMapExt hardwareMap) {
        this.hardware = hardwareMap;
        this.m_driveTrain = new DriveTrain(hardware);
        this.m_intake = new Intake(hardware);

        setRobotState(RobotState.INIT);
        setGameState(GameState.IDLE);
    }

    public void setRobotState(RobotState newState) {
        if (this.currentRobotState == newState) return;

        handleRobotStateExit(this.currentRobotState);
        this.currentRobotState = newState;
        handleRobotStateEntry(newState);
    }

    public void setGameState(GameState newState) {
        if (this.currentGameState == newState) return;

        handleGameStateExit(this.currentGameState);
        this.currentGameState = newState;
        handleGameStateEntry(newState);
    }

    private void handleGameStateExit(GameState oldState) {
        // Clean up previous game state
        switch (oldState) {
            case INTAKING:
                m_intake.setIntakeState(Intake.IntakeState.IDLE);
                break;
            case SCORING:
                // Retract scoring mechanism, etc.
                break;
        }
    }

    private void handleGameStateEntry(GameState newState) {
        // Configure subsystems for new game state
        switch (newState) {
            case INTAKING:
                // Start intake motors, lower intake, etc.
                m_driveTrain.setDriveState(DriveTrain.DriveState.NORMAL);
                m_intake.setIntakeState(Intake.IntakeState.INTAKE);
                break;
            case EXTAKING:
                // Start intake motors, lower intake, etc.
                m_driveTrain.setDriveState(DriveTrain.DriveState.NORMAL);
                m_intake.setIntakeState(Intake.IntakeState.EXTAKE);
                break;
            case SCORING:
                // Prepare scoring mechanism, precision drive
                m_driveTrain.setDriveState(DriveTrain.DriveState.PRECISION);
                m_intake.setIntakeState(Intake.IntakeState.IDLE);
                break;
            case LIFTING:
                // Slow, careful movements for climbing
                m_driveTrain.setDriveState(DriveTrain.DriveState.STOP);
                m_intake.setIntakeState(Intake.IntakeState.IDLE);
                break;
            case IDLE:
                // Aggressive drive mode
                m_driveTrain.setDriveState(DriveTrain.DriveState.STOP);
                m_intake.setIntakeState(Intake.IntakeState.IDLE);
                break;
        }
    }
    private void handleRobotStateExit(RobotState oldState) {
        switch (oldState) {
            case TELEOP:
                m_driveTrain.stop();
                break;
            case ESTOP:
                break;
        }
    }

    private void handleRobotStateEntry(RobotState newState) {
        switch (newState) {
            case TELEOP:
                m_driveTrain.setDriveState(DriveTrain.DriveState.NORMAL);
                m_intake.setIntakeState(Intake.IntakeState.IDLE);
                break;
            case ESTOP:
                m_driveTrain.setDriveState(DriveTrain.DriveState.STOP);
                m_intake.setIntakeState(Intake.IntakeState.STOP);
                break;
        }
    }

    private void handleStatePeriodic() {
        switch (currentGameState) {
            case INTAKING:
                // Run intake logic, check if we have element, etc.
                break;
            case SCORING:
                // Run scoring sequence, check alignment, etc.
                break;
            case LIFTING:
                // Climbing sequence, check hooks, etc.
                break;
        }
    }


    public void emergencyStop() {
        setRobotState(RobotState.ESTOP);
        setGameState(GameState.IDLE);
    }

    public DriveTrain getDriveTrain() {
        return m_driveTrain;
    }
    public Intake getIntake() {                            
        return m_intake;
    }
    public RobotState getCurrentRobotState() {
        return currentRobotState;
    }


    public hwMapExt getHardware() {
        return hardware;
    }

}
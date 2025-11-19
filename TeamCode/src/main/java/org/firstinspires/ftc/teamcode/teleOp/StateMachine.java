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
    
    private boolean endgameTriggered = false;
    private double matchTime = 0;

    public StateMachine(hwMapExt hardwareMap) {
        this.hardware = hardwareMap;
        this.m_driveTrain = new DriveTrain(hardware);
        this.m_intake = new Intake(hardware);

        setRobotState(RobotState.DISABLED);
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
            case CLIMBING:
                // Slow, careful movements for climbing
                m_driveTrain.setDriveState(DriveTrain.DriveState.PRECISION);
                m_intake.setIntakeState(Intake.IntakeState.IDLE);
                break;
            case DEFENDING:
                // Aggressive drive mode
                m_driveTrain.setDriveState(DriveTrain.DriveState.TURBO);
                m_intake.setIntakeState(Intake.IntakeState.IDLE);
                break;
        }
    }

    public void checkRobotStateTransitions(double matchTime) {
        this.matchTime = matchTime;
        switch (currentRobotState) {
            case TELEOP:
                // Teleop-specific updates if needed
                break;
            case AUTONOMOUS:
                // Autonomous-specific updates if needed
                break;
            case DISABLED:
                // Disabled-specific updates if needed
                break;
            case ESTOP:
                // E-stop handling
                break;
        }
    }

    private void handleRobotStateExit(RobotState oldState) {
        switch (oldState) {
            case TELEOP:
                m_driveTrain.stop();
                break;
            case AUTONOMOUS:
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
            case DISABLED:
                m_driveTrain.setDriveState(DriveTrain.DriveState.STOP);
                break;
            case ESTOP:
                m_driveTrain.setDriveState(DriveTrain.DriveState.STOP);
                m_intake.setIntakeState(Intake.IntakeState.STOP);
                break;
        }
    }

    public void update(double matchTime) {
        this.matchTime = matchTime;

        checkRobotStateTransitions();
        checkGameStateTransitions();
        handleStatePeriodic();
    }

    private void checkRobotStateTransitions() {
        if (currentRobotState == RobotState.TELEOP && matchTime <= 30 && !endgameTriggered) {
            setRobotState(RobotState.ENDGAME);
            endgameTriggered = true;
        }
    }

    private void checkGameStateTransitions() {
        // Game logic
        if (currentRobotState == RobotState.TELEOP || currentRobotState == RobotState.ENDGAME) {
            // game-specific transition logic
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
            case CLIMBING:
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

    public GameState getGameState() { return currentGameState; }

    public hwMapExt getHardware() {
        return hardware;
    }

}
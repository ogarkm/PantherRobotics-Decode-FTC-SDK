package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hware.hwMapExt;
public class DriveTrain {
    private final hwMapExt hardware;
    public enum DriveState {
        NORMAL,
        TURBO,
        PRECISION,
        STOP
    }
    private DriveState currentState = DriveState.NORMAL;
    private double speedMultiplier = 1.0;
    public DriveTrain(hwMapExt hardware) {
        this.hardware = hardware;
    }
    public void teleopDrive(double x, double y, double rx) {
        double denominator = ((Math.abs(y) + Math.abs(x) + Math.abs(rx)) > 1 ?
                (Math.abs(y) + Math.abs(x) + Math.abs(rx)) : 1) * speedMultiplier;
        double frontLeftPower = (y + x + rx) * denominator;
        double backLeftPower = (y - x + rx) * denominator;
        double frontRightPower = (y - x - rx) * denominator;
        double backRightPower = (y + x - rx) * denominator;

        hardware.setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    public void stop() {
        if (currentState != DriveState.STOP) {
            setDriveState(DriveState.STOP);
        }
        hardware.stopMotors();
    }
    public double getSpeed() {
        return speedMultiplier;
    }
    public void setDriveState(DriveState state) {
        this.currentState = state;
        switch (state) {
            case PRECISION:
                speedMultiplier = 0.3;
                break;
            case TURBO:
                speedMultiplier = 1.0;
                break;
            case NORMAL:
                speedMultiplier = 0.6;
                break;
            case STOP:
                speedMultiplier = 0.0;
                stop();
                break;
        }
    }
    public DriveState getDriveState() {
        return currentState;
    }
}

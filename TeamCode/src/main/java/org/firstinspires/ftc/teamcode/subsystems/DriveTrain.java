package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.Hware.hwMap;
public class DriveTrain {
    private final hwMap hardware;
    public enum DriveState {
        NORMAL,
        TURBO,
        PRECISION,
        CHARACTERIZATION, // For Road Runner tuning
        STOP
    }
    private DriveState currentState = DriveState.NORMAL;
    private double maxSpeed = 1.0;

    public DriveTrain(hwMap hardware) {
        this.hardware = hardware;
    }

    public void teleopDrive(double x, double y, double rx) {
        double denominator = ((Math.abs(y) + Math.abs(x) + Math.abs(rx)) > 1 ?
                (Math.abs(y) + Math.abs(x) + Math.abs(rx)) : 1) *
                (currentState == DriveState.TURBO ? 1.5 :
                        currentState == DriveState.PRECISION ? 3.5 :
                                currentState == DriveState.NORMAL ? 2.5: 2.0);

        double frontLeftPower = (y + x + rx) / denominator * maxSpeed;
        double backLeftPower = (y - x + rx) / denominator * maxSpeed;
        double frontRightPower = (y - x - rx) / denominator * maxSpeed;
        double backRightPower = (y + x - rx) / denominator * maxSpeed;

        hardware.setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void setCharacterizationVoltages(double flVolts, double frVolts, double blVolts, double brVolts) {
        if (currentState != DriveState.CHARACTERIZATION) {
            setDriveState(DriveState.CHARACTERIZATION);
        }

        // Convert voltages to motor powers (assuming 12V battery)
        double flPower = flVolts / 12.0;
        double frPower = frVolts / 12.0;
        double blPower = blVolts / 12.0;
        double brPower = brVolts / 12.0;

        hardware.setMotorPowers(flPower, frPower, blPower, brPower);
    }

    public void stop() {
        if (currentState != DriveState.STOP) {
            setDriveState(DriveState.STOP);
        }

        hardware.stopMotors();
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setDriveState(DriveState state) {
        this.currentState = state;

        switch (state) {
            case CHARACTERIZATION:
                hardware.setMotorModes(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case PRECISION:
                hardware.setMotorModes(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case TURBO:
                hardware.setMotorModes(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case NORMAL:
                hardware.setMotorModes(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case STOP:
                break;
        }
    }

    public DriveState getDriveState() {
        return currentState;
    }

}

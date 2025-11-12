package org.firstinspires.ftc.teamcode.HwMap;

import static org.firstinspires.ftc.teamcode.HwMap.Constants.DriveConstants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HwMap.Constants;

public class hwMap {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;


    //intake

    public DcMotor intake;
    public DcMotor rlauncher;
    public DcMotor llauncher;

    public hwMap(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get(DriveConstants.FRONT_LEFT_MOTOR);
        backLeftMotor = hardwareMap.dcMotor.get(DriveConstants.BACK_LEFT_MOTOR);
        frontRightMotor = hardwareMap.dcMotor.get(DriveConstants.FRONT_RIGHT_MOTOR);
        backRightMotor = hardwareMap.dcMotor.get(DriveConstants.BACK_RIGHT_MOTOR);

        frontLeftMotor.setDirection(DriveConstants.FRONT_LEFT_DIRECTION);
        backLeftMotor.setDirection(DriveConstants.BACK_LEFT_DIRECTION);
        frontRightMotor.setDirection(DriveConstants.FRONT_RIGHT_DIRECTION);
        backRightMotor.setDirection(DriveConstants.BACK_RIGHT_DIRECTION);

        //intake
        intake = hardwareMap.dcMotor.get(DriveConstants.INTAKE);

        intake.setDirection(DriveConstants.INTAKE_DIR);


        rlauncher = hardwareMap.dcMotor.get(DriveConstants.RIGHT_LAUNCHER);
        llauncher = hardwareMap.dcMotor.get(DriveConstants.LEFT_LAUNCHER);

        rlauncher.setDirection(DriveConstants.R_LAUNCH_DIR);
        llauncher.setDirection(DriveConstants.F_LAUNCH_DIR);


        setMotorZero(DriveConstants.MOTOR_ZERO_POWER_BEHAVIOR);
        setMotorModes(DriveConstants.MOTOR_RUNMODE);
    }

    public void setMotorModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    public void setMotorZero(DcMotor.ZeroPowerBehavior zero) {
        frontLeftMotor.setZeroPowerBehavior(zero);
        backLeftMotor.setZeroPowerBehavior(zero);
        frontRightMotor.setZeroPowerBehavior(zero);
        backRightMotor.setZeroPowerBehavior(zero);
    }

    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void setIntakePower(double intakePower) {
        frontLeftMotor.setPower(intakePower);
    }

    public void setLauncherPower(double launchPower) {
        llauncher.setPower(launchPower);
        rlauncher.setPower(launchPower);
    }

    public void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void resetEncoders() {
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int[] getEncoderPositions() {
        return new int[] {
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(),
                backLeftMotor.getCurrentPosition(),
                backRightMotor.getCurrentPosition()
        };
    }
}
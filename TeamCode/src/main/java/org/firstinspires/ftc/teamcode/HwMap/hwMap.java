package org.firstinspires.ftc.teamcode.HwMap;

import static org.firstinspires.ftc.teamcode.HwMap.Constants.DriveConstants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HwMap.Constants;

public class hwMap {
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;


    //intake
    public DcMotor intake;
    public DcMotor rlauncher;
    public DcMotor llauncher;

    public hwMap(HardwareMap hardwareMap) {
        frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get(DriveConstants.FRONT_LEFT_MOTOR);
        backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get(DriveConstants.BACK_LEFT_MOTOR);
        frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get(DriveConstants.FRONT_RIGHT_MOTOR);
        backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get(DriveConstants.BACK_RIGHT_MOTOR);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //intake
        intake = hardwareMap.dcMotor.get(DriveConstants.INTAKE);

        intake.setDirection(DriveConstants.INTAKE_DIR);


        rlauncher = hardwareMap.dcMotor.get(Constants.LaunchConstants.RIGHT_LAUNCHER);
        llauncher = hardwareMap.dcMotor.get(Constants.LaunchConstants.LEFT_LAUNCHER);

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
        intake.setPower(intakePower);
    }

    public void setLauncherPower(double launchPower) {
        llauncher.setPower(launchPower);
        rlauncher.setPower(launchPower);
    }


}
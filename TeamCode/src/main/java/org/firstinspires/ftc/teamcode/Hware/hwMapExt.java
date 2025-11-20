package org.firstinspires.ftc.teamcode.Hware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleOp.Constants;

public class hwMapExt {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;


    public DcMotor frontIntakeMotor;
    public DcMotor backIntakeMotor;

    //looking at it from the back (front is facing the front)
    public Servo ptoLeft;
    public Servo ptoRight;

    public hwMapExt(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");

        ptoLeft = hardwareMap.servo.get(Constants.LiftConstants.PTO_LEFT);
        ptoRight = hardwareMap.servo.get(Constants.LiftConstants.PTO_RIGHT);

        ptoRight.setDirection(Constants.LiftConstants.CW);
        ptoLeft.setDirection(Constants.LiftConstants.CCW);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontIntakeMotor = hardwareMap.dcMotor.get(Constants.IntakeConstants.FRONT_INTAKE_MOTOR);
        backIntakeMotor = hardwareMap.dcMotor.get(Constants.IntakeConstants.BACK_INTAKE_MOTOR);

        frontIntakeMotor.setDirection(Constants.IntakeConstants.FRONT_INTAKE_DIRECTION);
        backIntakeMotor.setDirection(Constants.IntakeConstants.BACK_INTAKE_DIRECTION);

        frontIntakeMotor.setZeroPowerBehavior(Constants.IntakeConstants.INTAKE_ZERO_POWER_BEHAVIOR);
        backIntakeMotor.setZeroPowerBehavior(Constants.IntakeConstants.INTAKE_ZERO_POWER_BEHAVIOR);

        frontIntakeMotor.setMode(Constants.IntakeConstants.INTAKE_RUNMODE);
        backIntakeMotor.setMode(Constants.IntakeConstants.INTAKE_RUNMODE);

    }
    private int inchesToTicks(double inches) {
        double wheelDiameter = Constants.DriveConstants.WHEEL_DIAMETER;  // in inches
        double ticksPerRev = Constants.DriveConstants.TICKS_PER_REVOLUTION;
        double gearRatio = 2.0;
        double circumference = Math.PI * wheelDiameter;

        return (int) (inches * (ticksPerRev * gearRatio) / circumference);
    }

    public void setServoPosition(double position){
        ptoLeft.setPosition(position);
        ptoRight.setPosition(position);
    }

    public void setMotorTargetPositions(double inches){
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setTargetPosition(-inchesToTicks(inches));
        frontRightMotor.setTargetPosition(inchesToTicks(inches));
        backLeftMotor.setTargetPosition(-inchesToTicks(inches));
        backRightMotor.setTargetPosition(inchesToTicks(inches));

    }
    public void setMotorModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    public void setMotorPowers(double frontLeftPower, double frontRightPower,
                               double backLeftPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        frontLeftMotor.setZeroPowerBehavior(mode);
        backLeftMotor.setZeroPowerBehavior(mode);
        frontRightMotor.setZeroPowerBehavior(mode);
        backRightMotor.setZeroPowerBehavior(mode);
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
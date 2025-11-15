package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HwMap.Constants;
import org.firstinspires.ftc.teamcode.HwMap.hwMap;

public class AutonMethods extends LinearOpMode {
    double ticks = Constants.DriveConstants.TICKS_PER_REVOLUTION;
    double tl = Constants.DriveConstants.APPROXIMATE_DISTANCE_PER_TILE;
    double newTarget;
    hwMap robot;
    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
    }
    public int getTicks(double distance) {
        return (int) (ticks * 2 * Math.PI * distance * 1);
    }
    public void drive(double d){

        robot = new hwMap(hardwareMap);
        newTarget=getTicks(d);


        robot.frontRightMotor.setTargetPosition((int)newTarget);
        robot.frontLeftMotor.setTargetPosition((int)newTarget);
        robot.backRightMotor.setTargetPosition((int)newTarget);
        robot.backLeftMotor.setTargetPosition((int)newTarget);

        robot.setMotorPowers(1,1,1,1);
        robot.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turn(double d){

        robot = new hwMap(hardwareMap);
        newTarget=getTicks(d);

        robot.frontRightMotor.setTargetPosition(-(int)newTarget);
        robot.frontLeftMotor.setTargetPosition((int)newTarget);
        robot.backRightMotor.setTargetPosition(-(int)newTarget);
        robot.backLeftMotor.setTargetPosition((int)newTarget);

        robot.setMotorPowers(1,1,1,1);
        robot.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void strafe(double d){

        robot = new hwMap(hardwareMap);
        newTarget=getTicks(d);

        robot.frontRightMotor.setTargetPosition(-(int)newTarget);
        robot.frontLeftMotor.setTargetPosition((int)newTarget);
        robot.backRightMotor.setTargetPosition((int)newTarget);
        robot.backLeftMotor.setTargetPosition(-(int)newTarget);

        robot.setMotorPowers(1,1,1,1);
        robot.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void shootOn(){

        robot = new hwMap(hardwareMap);
        robot.setLauncherPower(.57);
    }

    public void shootOff(){

        robot = new hwMap(hardwareMap);
        robot.setLauncherPower(0);
        robot.setIntakePower(0);
    }

    public void intakeOn(){

        robot = new hwMap(hardwareMap);
        robot.setLauncherPower(-0.1);
        robot.setIntakePower(1);
    }
    public void intakeOff(){

        robot = new hwMap(hardwareMap);
        robot.setLauncherPower(0);
        robot.setIntakePower(0);
    }
}
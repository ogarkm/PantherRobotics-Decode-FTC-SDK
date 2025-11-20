package org.firstinspires.ftc.teamcode.Hware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class hwMapExt {

    private final HardwareMap hardwareMapObj;



    private static final boolean USE_WEBCAM = true;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;


    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    public DcMotor turretLeftMotor;
    public DcMotor turretRightMotor;

    public Servo turretservo;
    public Servo hoodservo;

    public hwMapExt(HardwareMap hardwareMap) {
        this.hardwareMapObj = hardwareMap;

        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretLeftMotor = hardwareMap.dcMotor.get(Constants.TurretConstants.TURRET_LEFT_MOTOR);
        turretRightMotor = hardwareMap.dcMotor.get(Constants.TurretConstants.TURRET_RIGHT_MOTOR);

        turretLeftMotor.setDirection(Constants.TurretConstants.TURRET_MOTOR_DIRECTION);
        turretRightMotor.setDirection(Constants.TurretConstants.TURRET_MOTOR_DIRECTION);

        turretservo = hardwareMap.servo.get(Constants.TurretConstants.LEFT_TURRET_SERVO);
        hoodservo = hardwareMap.servo.get(Constants.TurretConstants.HOOD_TURRET_SERVO);
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

    public void setTurretPower(double power) {
        turretLeftMotor.setPower(power);
        turretRightMotor.setPower(power);
    }

    public void turretOff() {
        turretLeftMotor.setPower(0);
        turretRightMotor.setPower(0);
    }

    public void setHoodPos(double pos) {
        hoodservo.setPosition(pos);
    }

    public void setTurretPos(double pos) {
        hoodservo.setPosition(pos);
    }

    public AprilTagDetection getAprilTagById(int targetTagId) {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.id == targetTagId) {
                return detection;
            }
        }

        return null;  // Not found
    }




    public void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMapObj.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }
}
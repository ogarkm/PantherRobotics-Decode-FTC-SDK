package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.HwMap.hwMap;
import org.firstinspires.ftc.teamcode.HwMap.Constants;

import java.util.List;

@TeleOp(name="Mecanum_Strafe_Test", group="TeleOp")
public class TeleOP extends LinearOpMode {

    hwMap hw;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private static final boolean USE_WEBCAM = true;

    private double launchPower = 0.0;
    private double launchMode;

    private static final double P_YAW_ROTATE = 0.9;
    private static final double YAW_DEADBAND = 1.5;
    private static final double MAX_ROTATE_POWER = 0.5;


    static final double TICKS_PER_REV = Constants.DriveConstants.TICKS_PER_REVOLUTION;
    static final double WHEEL_DIAMETER_IN = Constants.DriveConstants.WHEEL_DIAMETER;
    static final double GEAR_RATIO = 1.0;


    @Override
    public void runOpMode() throws InterruptedException {
        hw = new hwMap(hardwareMap);
        initAprilTag();
        waitForStart();

        telemetry.addLine("TeleOp Active");
        telemetry.update();

        launchPower = Constants.LaunchConstants.LAUNCH_MID;

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            drive(x, y, rx);

            if (gamepad1.dpad_up) {
                driveInches(12, 0.5);
            }

            if (gamepad1.dpad_down) {
                driveInches(-12, 0.5);
            }

            if (gamepad1.right_trigger > 0) intake(1);
            else if (gamepad1.left_trigger > 0) intake(-1);
            else intake(0);


            if (gamepad1.a && launchPower == 0.0) launchPower = launchMode;
            else if (gamepad1.a && launchPower != 0.0) launchPower = 0.0;
            launch(launchPower);


            if (gamepad1.right_stick_button) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                AprilTagDetection targetTag = null;

                for (AprilTagDetection detection : detections) {
                    if (detection.metadata != null &&
                            detection.id == Constants.FieldConstants.redAprilTagID) {
                        targetTag = detection;
                        break;
                    }
                }

                if (targetTag != null) {
                    double rawYaw = targetTag.ftcPose.yaw;
                    double yawDeg = (Math.abs(rawYaw) <= 2 * Math.PI + 1e-6)
                            ? Math.toDegrees(rawYaw)
                            : rawYaw;

                    if (Math.abs(yawDeg) <= YAW_DEADBAND) {
                        drive(0, 0, 0);
                        telemetry.addData("AutoRotate", "ON TARGET %.2f deg", yawDeg);
                    } else {
                        double rxPower = -yawDeg * P_YAW_ROTATE / 100.0;
                        rxPower = Range.clip(rxPower, -MAX_ROTATE_POWER, MAX_ROTATE_POWER);
                        drive(0, 0, rxPower);
                        telemetry.addData("AutoRotate", "Turning %.2f°", yawDeg);
                    }
                } else {
                    drive(0, 0, 0);
                    telemetry.addData("AutoRotate", "No Tag");
                }
            }


            if (gamepad1.x) launchMode = Constants.LaunchConstants.LAUNCH_MAX;
            if (gamepad1.y) launchMode = Constants.LaunchConstants.LAUNCH_MID;
            if (gamepad1.b) launchMode = Constants.LaunchConstants.LAUNCH_LOW;

            telemetry.update();
        }
    }

    private int inchesToTicks(double inches) {
        double circumference = Math.PI * WHEEL_DIAMETER_IN;
        return (int)((inches / circumference) * TICKS_PER_REV * GEAR_RATIO);
    }

    private void driveInches(double inches, double power) {
        int ticks = inchesToTicks(inches);

        hw.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hw.frontLeftMotor.setTargetPosition(ticks);
        hw.frontRightMotor.setTargetPosition(ticks);
        hw.backLeftMotor.setTargetPosition(ticks);
        hw.backRightMotor.setTargetPosition(ticks);

        hw.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hw.setMotorPowers(power, power, power, power);

        while (opModeIsActive() &&
                (hw.frontLeftMotor.isBusy() || hw.frontRightMotor.isBusy() ||
                        hw.backLeftMotor.isBusy()  || hw.backRightMotor.isBusy())) {

            telemetry.addData("Driving", "%.1f inches", inches);
            telemetry.update();
        }

        hw.setMotorPowers(0, 0, 0, 0);

        hw.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    private void drive(double x, double y, double rx) {
        double fl =  (y + x + rx);
        double bl =  (y - x + rx);
        double fr =  (y - x - rx);
        double br =  (y + x - rx);

        hw.setMotorPowers(fl, fr, bl, br);
    }

    private void intake(double power) {
        hw.setIntakePower(power);
    }

    private void launch(double power) {
        hw.setLauncherPower(power);
    }
}

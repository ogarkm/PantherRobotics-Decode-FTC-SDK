package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HwMap.Constants;
import org.firstinspires.ftc.teamcode.HwMap.hwMap;

@TeleOp(name="Mecanum_Strafe_Test", group="TeleOp")
public class TeleOP extends LinearOpMode {

    hwMap hw;

    private double launchPower = 0.0;
    private double launchMode;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new hwMap(hardwareMap);

        telemetry.addLine("Click A on TeleOP start to switch to unlinked mode");
        telemetry.update();

        launchMode = Constants.LaunchConstants.LAUNCH_MID;
        launchPower = launchMode;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            drive(x, y, rx);

            // intake
            if (gamepad2.right_trigger > 0) {
                intake(1);
            } else if (gamepad2.left_trigger > 0) {
                intake(-1);
            } else {
                intake(0);
            }

            // launch toggle
            if (gamepad2.a && launchPower == 0.0) {
                launchPower = launchMode;
            }
            else{
                launchPower = 0.0;
            }
            launch(launchPower);

            // set launch mode
            if (gamepad2.x) launchMode = Constants.LaunchConstants.LAUNCH_MID;
            if (gamepad2.b) launchMode = Constants.LaunchConstants.LAUNCH_LOW;
        }
    }

    private void drive(double x, double y, double rx) {
        double frontLeftPower = (y + x + rx);
        double backLeftPower = (y - x + rx);
        double frontRightPower = (y - x - rx);
        double backRightPower = (y + x - rx);

        hw.setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void intake(double power) {
        hw.setIntakePower(power);
    }

    private void launch(double power) {
        hw.setLauncherPower(power);
    }
}

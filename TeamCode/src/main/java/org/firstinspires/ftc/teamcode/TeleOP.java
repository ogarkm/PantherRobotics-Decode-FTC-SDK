// #---------------------------#
//  Basic Mecanum TeleOP
// 
//  Adapted from Game Manual 0
//  https://gm0.org
//  
// #---------------------------#

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwMap.Constants;
import org.firstinspires.ftc.teamcode.HwMap.hwMap;

@TeleOp(name="Mecanum_Strafe_Test", group="TEST")
public class TeleOP extends LinearOpMode {

    hwMap hw = new hwMap(hardwareMap);

    // Mode = 1 = Linked Mode (Intake and Launcher)
    // Mode = 2 = UnLinked Mode

    public int mode = 1;

    private double launchPower;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        telemetry.addLine("Click A on TeleOP start to switch to unlinked mode");
        telemetry.update();

        launchPower = Constants.DriveConstants.LAUNCH_MID;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            drive(x, y, rx);

            //intake
            if (gamepad1.right_trigger > 0) {
                intake(1);
            }
            else if (gamepad1.left_trigger > 0) {
                intake(-1);
            }
            //launch
            if (mode == 1 && (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0)) {
                launch(launchPower);
            }
            else if (mode == 2 && gamepad1.a) {
                launch(launchPower);
            }


            if (gamepad1.x) {launchPower = Constants.DriveConstants.LAUNCH_MAX;}
            if (gamepad1.y) {launchPower = Constants.DriveConstants.LAUNCH_MID;}
            if (gamepad1.b) {launchPower = Constants.DriveConstants.LAUNCH_LOW;}


            if (gamepad1.start) { if (mode == 1) {mode = 2;} else {mode = 1;} } //mode switcher



        }
    }

    private void drive(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1) * 2;
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        hw.setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void intake(double power) {
        hw.setIntakePower(power);
    }

    private void launch(double power) {
        hw.setLauncherPower(power);
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HwMap.Constants;
import org.firstinspires.ftc.teamcode.HwMap.hwMap;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "RedObsAuto")
public class BlueObsAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        hwMap hw = new hwMap(hardwareMap);
        waitForStart();
        hw.drive(-20);
        sleep(2000);
        hw.shootOn();
        sleep(3000);
        hw.intakeOn();
        sleep(700);
        hw.intakeOff();
        sleep(500);
        hw.intakeOn();
        sleep(500);
        hw.intakeOff();
        sleep(500);
        hw.intakeOn();
        sleep(500);
        hw.intakeOff();
        sleep(500);
        hw.shootOff();
    }
}
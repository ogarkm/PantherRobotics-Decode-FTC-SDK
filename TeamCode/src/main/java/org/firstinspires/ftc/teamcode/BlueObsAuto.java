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

@Autonomous(name = "BlueObsAuto")
public class BlueObsAuto extends LinearOpMode {
    AutonMethods auto;
    @Override
    public void runOpMode() throws InterruptedException{
        auto = new AutonMethods();
        waitForStart();

        //auto.strafe(10);
        auto.turn(10);
        sleep(1000);
        /*auto.turn(30);
        auto.shootOn();
        sleep(3000);
        auto.intakeOn();
        sleep(100);
        auto.intakeOff();
        auto.intakeOn();
        sleep(100);
        auto.intakeOff();
        auto.intakeOn();
        sleep(100);
        auto.intakeOff();
        sleep(500);
        auto.shootOff();*/
    }
}
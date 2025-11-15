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

@Autonomous(name = "RedNet")
public class RedNet extends LinearOpMode {
    hwMap robot;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new hwMap(hardwareMap);
        waitForStart();

        robot.drive(62.9523025);
        sleep(2000);
        robot.turn(-20);
        robot.strafe(10);
        robot.setIntakePower(1);
        robot.drive(40);
        robot.intakeOn(); // currently has the launcher on very low power running in reverse to shots...
        robot.intakeOff();
        //this will be done as we test
    }
}
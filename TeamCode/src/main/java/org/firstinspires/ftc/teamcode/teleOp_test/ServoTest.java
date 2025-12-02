package org.firstinspires.ftc.teamcode.teleOp_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Tuner", group="TEST")
public class ServoTest extends LinearOpMode {

    // Declare servos
    private Servo servo1;
    private Servo servo2;
    private Servo claw;
    private Servo armServo;

    @Override
    public void runOpMode() {

        // ---- GET SERVOS FROM HARDWARE MAP ----
        servo1   = hardwareMap.get(Servo.class, "servo1");
        servo2   = hardwareMap.get(Servo.class, "servo2");
        claw     = hardwareMap.get(Servo.class, "claw");
        armServo = hardwareMap.get(Servo.class, "armServo");

        // Array for easy tuning
        Servo[] servos = { servo1, servo2, claw, armServo };
        String[] names = { "servo1", "servo2", "claw", "armServo" };

        double[] pos = { 0.5, 0.5, 0.5, 0.5 };

        int index = 0;

        telemetry.addLine("Servo Tuner Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---- SWITCH SERVO ----
            if (gamepad1.dpad_up) {
                index = (index + 1) % servos.length;
                sleep(200);
            }
            if (gamepad1.dpad_down) {
                index = (index - 1 + servos.length) % servos.length;
                sleep(200);
            }

            // ---- POSITION CONTROL ----
            if (gamepad1.left_bumper) pos[index] -= 0.01;
            if (gamepad1.right_bumper) pos[index] += 0.01;

            pos[index] -= gamepad1.left_trigger * 0.10;
            pos[index] += gamepad1.right_trigger * 0.10;

            // Clamp 0–1
            pos[index] = Math.max(0.0, Math.min(1.0, pos[index]));
            servos[index].setPosition(pos[index]);

            // ---- TOGGLE DIRECTION ----
            if (gamepad1.x) {
                if (servos[index].getDirection() == Servo.Direction.FORWARD)
                    servos[index].setDirection(Servo.Direction.REVERSE);
                else
                    servos[index].setDirection(Servo.Direction.FORWARD);

                sleep(200);
            }

            // ---- TELEMETRY ----
            telemetry.addLine("=== Servo Tuner ===");
            telemetry.addData("Selected Servo", names[index]);
            telemetry.addData("Position", "%.3f", pos[index]);
            telemetry.addData("Direction", servos[index].getDirection());
            telemetry.addLine("Controls:");
            telemetry.addLine("- DPAD Up/Down: Change servo");
            telemetry.addLine("- LB/RB: +/- 0.01");
            telemetry.addLine("- LT/RT: +/- 0.10");
            telemetry.addLine("- X: Toggle direction");
            telemetry.update();
        }
    }
}

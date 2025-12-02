package org.firstinspires.ftc.teamcode.teleOp_test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="Color Sensor Readout", group="TEST")
public class ColorSensorTest extends LinearOpMode {

    private ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(ColorSensor.class, "color"); // name in config

        float[] hsv = new float[3];

        telemetry.addLine("Color Sensor Test Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Read RGB
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();
            int a = colorSensor.alpha();

            // Convert to HSV for better color classification
            android.graphics.Color.RGBToHSV(r, g, b, hsv);

            String colorName = classifyColor(hsv, r, g, b, a);

            telemetry.addData("Raw Red", r);
            telemetry.addData("Raw Green", g);
            telemetry.addData("Raw Blue", b);
            telemetry.addData("Alpha", a);

            telemetry.addData("HSV - H", hsv[0]);
            telemetry.addData("HSV - S", hsv[1]);
            telemetry.addData("HSV - V", hsv[2]);

            telemetry.addData("Detected Color", colorName);

            telemetry.update();
        }
    }

    /** Simple heuristic color classifier */
    private String classifyColor(float[] hsv, int r, int g, int b, int alpha) {
        float H = hsv[0];
        float S = hsv[1];
        float V = hsv[2];

        if (V < 0.1) return "Black";
        if (V > 0.8 && S < 0.2) return "White";

        // Very rough hue-based classification
        if      (H < 15 || H > 340) return "Red";
        else if (H < 60)            return "Yellow";
        else if (H < 170)           return "Green";
        else if (H < 255)           return "Blue";
        else                        return "Purple";
    }
}

package org.firstinspires.ftc.teamcode;

import android.app.Activity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import android.graphics.Color;
import android.view.View;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class PixelDetection extends LinearOpMode {
    NormalizedColorSensor colorSensor;
    View relativeLayout;
    public static float gain = 1f;
    @Override
    public void runOpMode() throws InterruptedException {

        final float[] hsvValues = new float[3];
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            colorSensor.setGain(gain);
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(),hsvValues);
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);
            if (hsvValues[1] <= 0.3)
                telemetry.addData("Color Detected Is: ", "White");
            else if (hsvValues[0] >= 20 && hsvValues[0] <= 70)
                telemetry.addData("Color Detected Is: ", "Yellow");
            else if (hsvValues[0] >= 90 && hsvValues[0] <= 130)
                telemetry.addData("Color Detected Is: ", "Green");
            else if (hsvValues[0] >= 240 && hsvValues[0] <= 290)
                telemetry.addData("Color Detected Is: ", "Purple");
            else
                telemetry.addLine("No color in valid range.");

            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import android.app.Activity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

@Config
@Autonomous
public class PixelDetection extends LinearOpMode {
    NormalizedColorSensor colorSensor;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern blinkinPattern;

    public static float gain = 0.8f;
    public static double whiteValue = 0.5;
    public static int yellowMin = 20;
    public static int yellowMax = 80;
    public static int greenMin = 100;
    public static int greenMax = 130;
    public static int purpleMin = 180;
    public static int purpleMax = 290;



    @Override
    public void runOpMode() throws InterruptedException {

        final float[] hsvValues = new float[3];
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver.setPattern(blinkinPattern);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            colorSensor.setGain(gain);
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            if (hsvValues[1] <= whiteValue)
                //telemetry.addData("Color Detected Is: ", "White");
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            else if (hsvValues[0] >= yellowMin && hsvValues[0] <= yellowMax)
                //telemetry.addData("Color Detected Is: ", "Yellow");
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            else if (hsvValues[0] >= greenMin && hsvValues[0] <= greenMax)
                //telemetry.addData("Color Detected Is: ", "Green");
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            else if (hsvValues[0] >= purpleMin && hsvValues[0] <= purpleMax)
                //telemetry.addData("Color Detected Is: ", "Purple");
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
            else
                //telemetry.addLine("No color in valid range.");
                blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.RED;


            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            blinkinLedDriver.setPattern(blinkinPattern);
            telemetry.addData("Color Is", blinkinPattern);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous
public class PixelDetection extends LinearOpMode {
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    RevBlinkinLedDriver blinkinLedDriver1;
    RevBlinkinLedDriver.BlinkinPattern blinkinPattern1;
    RevBlinkinLedDriver blinkinLedDriver2;
    RevBlinkinLedDriver.BlinkinPattern blinkinPattern2;

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

        final float[] hsvValues1 = new float[3];
        final float[] hsvValues2 = new float[3];
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor2");
        if (colorSensor1 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor1).enableLight(true);
        }
        if (colorSensor2 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor2).enableLight(true);
        }

        blinkinLedDriver1 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin1");
        blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver1.setPattern(blinkinPattern1);

        blinkinLedDriver2 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin2");
        blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver2.setPattern(blinkinPattern2);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
            Color.colorToHSV(colors1.toColor(), hsvValues2);
            telemetry.addLine()
                    .addData("Red", "%.3f", colors1.red)
                    .addData("Green", "%.3f", colors1.green)
                    .addData("Blue", "%.3f", colors1.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues1[0])
                    .addData("Saturation", "%.3f", hsvValues1[1])
                    .addData("Value", "%.3f", hsvValues1[2]);
            telemetry.addData("Alpha", "%.3f", colors1.alpha);
            colorSensor2.setGain(gain);
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
            Color.colorToHSV(colors2.toColor(), hsvValues2);
            telemetry.addLine()
                    .addData("Red", "%.3f", colors2.red)
                    .addData("Green", "%.3f", colors2.green)
                    .addData("Blue", "%.3f", colors2.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues2[0])
                    .addData("Saturation", "%.3f", hsvValues2[1])
                    .addData("Value", "%.3f", hsvValues2[2]);
            telemetry.addData("Alpha", "%.3f", colors2.alpha);

            if (hsvValues1[1] <= whiteValue) {
                blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                if (hsvValues2[1] <= whiteValue)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                else if (hsvValues2[0] >= yellowMin && hsvValues2[0] <= yellowMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                else if (hsvValues2[0] >= greenMin && hsvValues2[0] <= greenMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                else if (hsvValues2[0] >= purpleMin && hsvValues2[0] <= purpleMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                else
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
            } else if (hsvValues2[0] >= yellowMin && hsvValues2[0] <= yellowMax) {
                blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                if (hsvValues2[1] <= whiteValue)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                else if (hsvValues2[0] >= yellowMin && hsvValues2[0] <= yellowMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                else if (hsvValues2[0] >= greenMin && hsvValues2[0] <= greenMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                else if (hsvValues2[0] >= purpleMin && hsvValues2[0] <= purpleMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                else
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
            }  else if (hsvValues2[0] >= greenMin && hsvValues2[0] <= greenMax) {
                blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                if (hsvValues2[1] <= whiteValue)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                else if (hsvValues2[0] >= yellowMin && hsvValues2[0] <= yellowMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                else if (hsvValues2[0] >= greenMin && hsvValues2[0] <= greenMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                else if (hsvValues2[0] >= purpleMin && hsvValues2[0] <= purpleMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                else
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
            } else if (hsvValues2[0] >= purpleMin && hsvValues2[0] <= purpleMax) {
                blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.RED;
                if (hsvValues2[1] <= whiteValue)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                else if (hsvValues2[0] >= yellowMin && hsvValues2[0] <= yellowMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                else if (hsvValues2[0] >= greenMin && hsvValues2[0] <= greenMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                else if (hsvValues2[0] >= purpleMin && hsvValues2[0] <= purpleMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                else
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
            } else {
                blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.RED;
                if (hsvValues2[1] <= whiteValue)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                else if (hsvValues2[0] >= yellowMin && hsvValues2[0] <= yellowMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                else if (hsvValues2[0] >= greenMin && hsvValues2[0] <= greenMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                else if (hsvValues2[0] >= purpleMin && hsvValues2[0] <= purpleMax)
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                else
                    blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
            }

            if (colorSensor2 instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM));
            }

            blinkinLedDriver2.setPattern(blinkinPattern2);
            telemetry.addData("Color Is", blinkinPattern2);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}

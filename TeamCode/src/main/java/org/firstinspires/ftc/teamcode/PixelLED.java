package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Config
public class PixelLED extends LinearOpMode {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private TouchSensor touch;

    @Override
    public void runOpMode() throws InterruptedException{
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();


        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver.setPattern(pattern);

        touch = hardwareMap.get(TouchSensor.class, "touch");

        while (opModeIsActive()) {
            if (currentGamepad.dpad_left){
                if(currentGamepad.a && !previousGamepad.a) {
                    if (!touch.isPressed()) {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;

                    } else {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    }
                }

                else if (currentGamepad.b && !previousGamepad.b) {
                    if (!touch.isPressed()) {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;

                    } else {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    }
                }
                else if (currentGamepad.x && !previousGamepad.x) {
                    if (!touch.isPressed()) {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;

                    } else {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    }
                }
                else if (currentGamepad.y && !previousGamepad.y) {
                    if (!touch.isPressed()) {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;

                    } else {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    }
                }
            }

            if (currentGamepad.dpad_right){
                if(currentGamepad.a && !previousGamepad.a) {
                    if (!touch.isPressed()) {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;

                    } else {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    }
                }

                else if (currentGamepad.b && !previousGamepad.b) {
                    if (!touch.isPressed()) {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;

                    } else {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    }
                }
                else if (currentGamepad.x && !previousGamepad.x) {
                    if (!touch.isPressed()) {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;

                    } else {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    }
                }
                else if (currentGamepad.y && !previousGamepad.y) {
                    if (!touch.isPressed()) {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;

                    } else {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    }
                }
            }
            blinkinLedDriver.setPattern(pattern);
            telemetry.addData("Color Is",pattern);

            if (touch.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }

            telemetry.update();
        }
    }
}

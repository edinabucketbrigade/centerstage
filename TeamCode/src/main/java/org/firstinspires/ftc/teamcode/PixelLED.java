package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Config
@TeleOp
public class PixelLED extends LinearOpMode {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver blinkinLedDriver2;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    RevBlinkinLedDriver.BlinkinPattern pattern2;
    private TouchSensor touch;

    @Override
    public void runOpMode() throws InterruptedException{
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();


        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver.setPattern(pattern);
        blinkinLedDriver2 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver2.setPattern(pattern2);

        //digital ports are higher number on config
        touch = hardwareMap.get(TouchSensor.class, "touch");

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.left_bumper){
                if(currentGamepad.a) {
                    if (!touch.isPressed())
                        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    else
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                }

                else if (currentGamepad.b) {
                    if (!touch.isPressed())
                        pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                    else
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                }
                else if (currentGamepad.x) {
                    if (!touch.isPressed())
                        pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                    else
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                }
                else if (currentGamepad.y) {
                    if (!touch.isPressed())
                        pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    else
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                }
            }

            if (currentGamepad.right_bumper){
                if(currentGamepad.a) {
                    if (!touch.isPressed())
                        pattern2 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    else
                        pattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
                }

                else if (currentGamepad.b) {
                    if (!touch.isPressed())
                        pattern2 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                    else
                        pattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
                }
                else if (currentGamepad.x) {
                    if (!touch.isPressed())
                        pattern2 = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                    else
                        pattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
                }
                else if (currentGamepad.y) {
                    if (!touch.isPressed())
                        pattern2 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    else
                        pattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
                }
            }
            blinkinLedDriver.setPattern(pattern);
            telemetry.addData("Right Color Is",pattern);
            blinkinLedDriver2.setPattern(pattern2);
            telemetry.addData("Left Color Is",pattern2);

            if (touch.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }

            telemetry.update();
        }
    }
}

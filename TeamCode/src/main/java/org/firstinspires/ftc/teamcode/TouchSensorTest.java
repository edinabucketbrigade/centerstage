package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TouchSensorTest extends LinearOpMode {

    /*
    Control Hub Portal
        Control Hub
            Digital Devices
                5 - REV Touch Sensor - touch_sensor

    REV Touch Sensor
    https://www.revrobotics.com/rev-31-1425/

    REV Magnetic Limit Switch
    ttps://www.revrobotics.com/rev-31-1462/
    */

    @Override
    public void runOpMode() {

        TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");

        telemetry.addData("Message", "Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean isPressed = touchSensor.isPressed();
            telemetry.addData("Pressed", isPressed);
            telemetry.update();
        }

    }

}
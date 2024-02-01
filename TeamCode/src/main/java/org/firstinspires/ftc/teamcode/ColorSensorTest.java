package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Disabled
public class ColorSensorTest extends LinearOpMode {

    /*
    Control Hub Portal
        Control Hub
            I2C
                2 - REV Color Sensor V3 - color_sensor

    REV Color Sensor V3
    https://www.revrobotics.com/rev-31-1557/
    */

    @Override
    public void runOpMode() {

        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        DistanceSensor distanceSensor = (DistanceSensor)colorSensor;

        telemetry.addData("Message", "Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double distance = distanceSensor.getDistance(DistanceUnit.MM);
            telemetry.addData("Distance", "%.0f mm", distance);
            telemetry.update();
        }

    }

}
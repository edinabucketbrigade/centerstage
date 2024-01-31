package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorTest extends LinearOpMode {

    /*
    Control Hub Portal
        Control Hub
            I2C
                3 - REV 2m Distance Sensor - distance_sensor

    REV 2m Distance Sensor
    https://www.revrobotics.com/rev-31-1505/
    */

    @Override
    public void runOpMode() {

        DistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");

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
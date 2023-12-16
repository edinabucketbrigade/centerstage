package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TeleOpO extends LinearOpMode {

    DigitalChannel digitalChannel;

    @Override
    public void runOpMode() {

        digitalChannel = hardwareMap.get(DigitalChannel.class, "beam break");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("state",digitalChannel.getState());

            telemetry.addData("Status", "Running");
            telemetry.update();

        }

    }

}
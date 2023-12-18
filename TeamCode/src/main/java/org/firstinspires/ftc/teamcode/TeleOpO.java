package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TeleOpO extends LinearOpMode {

    DigitalChannel beamBreak;

    @Override
    public void runOpMode() {

        beamBreak = hardwareMap.get(DigitalChannel.class, "beam_break");

        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean beamBroken = !beamBreak.getState(); //May show status as true if beam is broken
            telemetry.addData("Beam Break Status", beamBroken);

            telemetry.addData("Status", "Running");
            telemetry.update();

        }

    }

}
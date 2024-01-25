package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
@Disabled
public class TeleOpO extends LinearOpMode {

    DigitalChannel beamBreak;

    @Override
    public void runOpMode() {

        beamBreak = hardwareMap.get(DigitalChannel.class, "beam_break");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean isItOpen = beamBreak.getState();
            String switchState;
            if (isItOpen) {
                switchState = "Open";
            } else {
                switchState = "Closed";
            }
            telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
            telemetry.addData("state", ":  " + switchState);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }

    }

}
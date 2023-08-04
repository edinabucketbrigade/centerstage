package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(preselectTeleOp = "TeleOpA")
public class AutoA extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addData("Running", "AutoA");
        telemetry.update();

    }

}
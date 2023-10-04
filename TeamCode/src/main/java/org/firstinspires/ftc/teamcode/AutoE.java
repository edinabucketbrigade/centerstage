package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(preselectTeleOp = "TeleOpE")
public class AutoE extends LinearOpMode {

    boolean red;
    boolean initialized = false;
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "X = blue, B = red");
        telemetry.update();

        while (!initialized) {
            if (gamepad1.x) {
                red = false;
                initialized = true;
            }
            else if (gamepad1.b) {
                red = true;
                initialized = true;
            }
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }

}
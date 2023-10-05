package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(preselectTeleOp = "TeleOpE")
public class AutoE extends LinearOpMode {

    Boolean red = null;
    Boolean left = null;
    Boolean direct = null;
    boolean initialized = false;
    @Override
    public void runOpMode() {

        boolean previousX = false;
        boolean previousB = false;

        while (!initialized) {

            boolean currentX = gamepad1.x;
            boolean currentB = gamepad1.b;

            if (red == null) {
                telemetry.addData("Status", "X = blue, B = red");
                telemetry.update();
                if (currentX && !previousX) {
                    red = false;
                }
                if (currentB && !previousB) {
                    red = true;
                }
            }
            else if (left == null){
                telemetry.addData("Status", "X = left, B = right");
                telemetry.update();
                if (currentX && !previousX) {
                    left = true;
                }
                if (currentB && !previousB) {
                    left = false;
                }
            }
            else if (direct == null) {
                telemetry.addData("Status", "X = direct, B = indirect");
                telemetry.update();
                if (currentX && !previousX) {
                    direct = true;
                }
                if (currentB && !previousB) {
                    direct = false;
                }
            }
            else {
                initialized = true;
            }
            previousX = currentX;
            previousB = currentB;
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Red", red);
        telemetry.addData("Left", left);
        telemetry.addData("Direct", direct);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }

}
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


        while (!initialized) {
            if (red == null) {
                telemetry.addData("Status", "X = blue, B = red");
                telemetry.update();
                if (gamepad1.x) {
                    red = false;
                } else if (gamepad1.b) {
                    red = true;
                }
            }
            else if (left == null){
                telemetry.addData("Status", "X = left, B = right");
                telemetry.update();
                if (gamepad1.x) {
                    left = true;
                } else if (gamepad1.b) {
                    left = false;
                }
            }
            else if (direct == null) {
                telemetry.addData("Status", "X = direct, B = indirect");
                telemetry.update();
                if (gamepad1.x) {
                    direct = true;
                } else if (gamepad1.b) {
                    direct = false;
                }
            }
            else {
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
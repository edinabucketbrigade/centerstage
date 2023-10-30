package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class TeleOpG extends LinearOpMode {

    private RobotHardwareA robotHardware = null;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robotHardware = new RobotHardwareA(this);

        boolean previousB = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousA = false;

        waitForStart();

        while (opModeIsActive()) {
            robotHardware.update();
            robotHardware.setDrivePowerAutomatically();

            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;
            boolean currentY = gamepad1.y;
            boolean currentA = gamepad1.a;

            if (currentY && !previousY) {
                robotHardware.toggleArm();
                robotHardware.toggleWrist();
            }

            if (currentX && !previousX) {
                robotHardware.toggleRightClaw();
            }

            if (currentB && !previousB) {
                robotHardware.toggleLeftClaw();
            }

            if (currentA && !previousA) {
                robotHardware.toggleWrist();
            }

            telemetry.addData("Status", "Running");

            telemetry.update();

            previousB = currentB;
            previousX = currentX;
            previousY = currentY;
            previousA = currentA;
        }

    }

}
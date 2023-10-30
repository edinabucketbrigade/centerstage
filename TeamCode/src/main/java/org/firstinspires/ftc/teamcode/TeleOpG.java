package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class TeleOpG extends LinearOpMode {

    private RobotHardwareA robotHardware = null;

    @Override
    public void runOpMode() {
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
                robotHardware.toggleLeftClaw();
            }

            if (currentB && !previousB) {
                robotHardware.toggleRightClaw();
            }

            if (currentA && !previousA) {
                robotHardware.toggleWrist();
            }

            previousB = currentB;
            previousX = currentX;
            previousY = currentY;
            previousA = currentA;
        }

    }

}
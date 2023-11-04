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

        telemetry.update();

        boolean previousB = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousA = false;
        boolean previousRightBumper = false;
        boolean previousDpadLeft = false;
        boolean previousDpadDown = false;

        waitForStart();

        while (opModeIsActive()) {

            robotHardware.update();

            robotHardware.moveRobot();

            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;
            boolean currentY = gamepad1.y;
            boolean currentA = gamepad1.a;
            boolean currentRightBumper = gamepad1.right_bumper;
            boolean currentDpadLeft = gamepad1.dpad_left;
            boolean currentDpadDown = gamepad1.dpad_down;

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

            if (currentRightBumper && !previousRightBumper) {
                robotHardware.toggleTurtleMode();
            }

            if (currentDpadLeft && !previousDpadLeft) {
                robotHardware.toggleFieldCentric();
            }

            if (currentDpadDown && !previousDpadDown) {
                robotHardware.toggleTelemetry();
            }

            previousB = currentB;
            previousX = currentX;
            previousY = currentY;
            previousA = currentA;
            previousRightBumper = currentRightBumper;
            previousDpadLeft = currentDpadLeft;
            previousDpadDown = currentDpadDown;

            telemetry.update();

        }

    }

}
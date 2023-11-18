package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;




@Config
@TeleOp
@Disabled
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
        boolean previousDpadLeft = false;

        waitForStart();

        while (opModeIsActive()) {

            robotHardware.update();

            robotHardware.moveRobot();

            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;
            boolean currentY = gamepad1.y;
            boolean currentA = gamepad1.a;
            boolean currentDpadLeft = gamepad1.dpad_left;

            if (currentY && !previousY) {
                robotHardware.toggleArm();
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

            robotHardware.setTurtleMode(gamepad1.left_bumper);

            if (currentDpadLeft && !previousDpadLeft) {
                robotHardware.toggleFieldCentric();
            }

            previousB = currentB;
            previousX = currentX;
            previousY = currentY;
            previousA = currentA;
            previousDpadLeft = currentDpadLeft;

            telemetry.update();

        }

    }

}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class WinchTest extends LinearOpMode {

    private RobotHardwareA robotHardware;

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardwareA(this);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                robotHardware.raiseWinchLift();
            }
            else if (gamepad1.a) {
                robotHardware.lowerWinchLift();
            }

            if (gamepad1.dpad_up) {
                robotHardware.raiseRobot();
            }
            else if (gamepad1.dpad_down) {
                robotHardware.lowerRobot();
            }
            else {
                robotHardware.stopLiftingRobot();
            }

        }

    }

}
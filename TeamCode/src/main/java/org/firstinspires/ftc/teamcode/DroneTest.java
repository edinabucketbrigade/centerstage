package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp
public class DroneTest extends LinearOpMode {

    private RobotHardwareA robotHardware;

    @Override
    public void runOpMode() {
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        robotHardware = new RobotHardwareA(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.b && !previousGamepad.b) {
                robotHardware.incrementDroneLaunchPosition();
            }

            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.decrementDroneLaunchPosition();
            }

            if (currentGamepad.y && !previousGamepad.y) {
                robotHardware.incrementDroneLiftPosition();
            }

            if (currentGamepad.x && !previousGamepad.x) {
                robotHardware.decrementDroneLiftPosition();
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Lift Position", "%.2f", robotHardware.getDroneLiftPosition());
            telemetry.addData("Launch Position", "%.2f", robotHardware.getDroneLaunchPosition());

            telemetry.update();
        }

    }

}
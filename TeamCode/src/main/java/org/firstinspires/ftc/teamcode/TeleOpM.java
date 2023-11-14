package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp
public class TeleOpM extends LinearOpMode {

    private RobotHardwareA robotHardware;
    private HeatSeek heatSeek;

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        robotHardware = new RobotHardwareA(this);
        heatSeek = new HeatSeek(robotHardware);

        robotHardware.isHighDrop = true;

        telemetry.update();

        waitForStart();

        robotHardware.resetYaw();

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            robotHardware.update();
            heatSeek.update();

            robotHardware.setTurtleMode(gamepad1.left_bumper);
            robotHardware.setBunnyMode(gamepad1.right_bumper);

            if (currentGamepad.options && !previousGamepad.options) {
                robotHardware.toggleFieldCentric();
            }

            if (currentGamepad.back && !previousGamepad.back) {
                robotHardware.resetYaw();
            }

            if (currentGamepad.x && !previousGamepad.x) {
                robotHardware.toggleLeftClaw();
            }

            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.toggleWrist();
            }

            if (currentGamepad.b && !previousGamepad.b) {
                robotHardware.toggleRightClaw();
            }

            if (currentGamepad.y && !previousGamepad.y){
                robotHardware.toggleArm();
            }

            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                heatSeek.startLeft();
            }

            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                heatSeek.startRight();
            }

            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                heatSeek.startMiddle();
            }

            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                heatSeek.cancel();
            }

            if (currentGamepad.right_trigger > 0.5 && previousGamepad.right_trigger <= 0.5) {
                robotHardware.toggleReverse();
            }

            telemetry.update();

        }
    }
}
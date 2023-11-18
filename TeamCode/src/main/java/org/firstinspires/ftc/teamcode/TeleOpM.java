package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp
public class TeleOpM extends LinearOpMode {

    private static final double TRIGGER_THRESHOLD = 0.5;
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

            if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
                robotHardware.toggleTurtleMode();
            }

            if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
                robotHardware.toggleBunnyMode();
            }

            if (currentGamepad.options && !previousGamepad.options) {
                robotHardware.toggleFieldCentric();
            }

            if (currentGamepad.back && !previousGamepad.back) {
                robotHardware.resetYaw();
            }

            boolean liftingRobot = false;

            if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
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
            }
            else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                if (currentGamepad.y) {
                    liftingRobot = true;
                    robotHardware.raiseRobot();
                }
                else if (currentGamepad.a) {
                    liftingRobot = true;
                    robotHardware.lowerRobot();
                }

                if (currentGamepad.x && !previousGamepad.x) {
                    robotHardware.launchDrone();
                }

                if (currentGamepad.b && !previousGamepad.b) {
                    robotHardware.toggleWinchLift();
                }
            }
            else {
                if (currentGamepad.a && !previousGamepad.a) {
                    robotHardware.toggleClaws();
                }

                if (currentGamepad.x && !previousGamepad.x) {
                    robotHardware.toggleLeftClaw();
                }

                if (currentGamepad.b && !previousGamepad.b) {
                    robotHardware.toggleRightClaw();
                }
            }

            if (!liftingRobot) {
                robotHardware.stopLiftingRobot();
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

            if (currentGamepad.left_stick_button && !previousGamepad.left_stick_button) {
                robotHardware.toggleReverse();
            }

            telemetry.update();

        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DroneTest extends LinearOpMode {
    private static final double INCREMENT = 0.05;

    private Servo droneLaunchServo;
    private Servo droneLiftServo;

    @Override
    public void runOpMode() {
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        double launchPosition = 0.8; // 0.6 - 1
        double liftPosition = 0.5; // 0.25 - 0.75

        droneLaunchServo = hardwareMap.get(Servo.class, "drone_launch_servo");
        droneLiftServo = hardwareMap.get(Servo.class, "drone_lift_servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        droneLaunchServo.setPosition(launchPosition);
        droneLiftServo.setPosition(liftPosition);

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.a && !previousGamepad.a) {
                launchPosition += INCREMENT;
                droneLaunchServo.setPosition(launchPosition);
            }

            if (currentGamepad.b && !previousGamepad.b) {
                launchPosition -= INCREMENT;
                droneLaunchServo.setPosition(launchPosition);
            }

            if (currentGamepad.y && !previousGamepad.y) {
                liftPosition += INCREMENT;
                droneLiftServo.setPosition(liftPosition);
            }

            if (currentGamepad.x && !previousGamepad.x) {
                liftPosition -= INCREMENT;
                droneLiftServo.setPosition(liftPosition);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Lift Position", "%.2f",liftPosition);
            telemetry.addData("Launch Position", "%.2f", launchPosition);

            telemetry.update();
        }

    }

}
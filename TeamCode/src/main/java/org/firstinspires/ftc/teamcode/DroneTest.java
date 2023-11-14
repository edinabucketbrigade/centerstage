package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class DroneTest extends LinearOpMode {
    public static double INCREMENT = 0.05;
    public static double LAUNCH_POSITION = 0.8; // 0.6 - 1
    public static double LIFT_POSITION = 0.5; // 0.25 - 0.75

    private Servo droneLaunchServo;
    private Servo droneLiftServo;

    @Override
    public void runOpMode() {
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        droneLaunchServo = hardwareMap.get(Servo.class, "drone_launch_servo");
        droneLiftServo = hardwareMap.get(Servo.class, "drone_lift_servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        droneLaunchServo.setPosition(LAUNCH_POSITION);
        droneLiftServo.setPosition(LIFT_POSITION);

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.b && !previousGamepad.b) {
                LAUNCH_POSITION += INCREMENT;
                droneLaunchServo.setPosition(LAUNCH_POSITION);
            }

            if (currentGamepad.a && !previousGamepad.a) {
                LAUNCH_POSITION -= INCREMENT;
                droneLaunchServo.setPosition(LAUNCH_POSITION);
            }

            if (currentGamepad.y && !previousGamepad.y) {
                LIFT_POSITION += INCREMENT;
                droneLiftServo.setPosition(LIFT_POSITION);
            }

            if (currentGamepad.x && !previousGamepad.x) {
                LIFT_POSITION -= INCREMENT;
                droneLiftServo.setPosition(LIFT_POSITION);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Lift Position", "%.2f",LIFT_POSITION);
            telemetry.addData("Launch Position", "%.2f", LAUNCH_POSITION);

            telemetry.update();
        }

    }

}
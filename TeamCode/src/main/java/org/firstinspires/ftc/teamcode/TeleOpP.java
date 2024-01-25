package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Disabled
/**
 * This class supports setting a servo's position.
 */
@Config
public class TeleOpP extends LinearOpMode {

    /**
     * Minimum servo position
     */
    public static double MINIMUM_POSITION = 0.5;

    /**
     * Maximum servo position
     */
    public static double MAXIMUM_POSITION = 0.6;

    /**
     * Runs the op mode.
     */
    @Override
    public void runOpMode() {

        // Get the servo.
        Servo servo = hardwareMap.get(Servo.class, "servo");

        // Display status.
        telemetry.addData("Status", "Initialized");

        // Update the telemetry.
        telemetry.update();

        // Wait for start.
        waitForStart();

        // While the op mode is active...
        while (opModeIsActive()) {

            // If the user pressed a...
            if(gamepad1.a) {

                // Set the minimum servo position.
                servo.setPosition(MINIMUM_POSITION);

            }

            // If the user pressed y...
            if(gamepad1.y) {

                // Set the maximum servo position.
                servo.setPosition(MAXIMUM_POSITION);

            }

            // Get the servo's position.
            double position = servo.getPosition();

            // Display status.
            telemetry.addData("Status", "Running");

            // Display the servo's position.
            telemetry.addData("Servo Position", "%.2f", position);

            // Update the telemetry.
            telemetry.update();

        }

    }

}
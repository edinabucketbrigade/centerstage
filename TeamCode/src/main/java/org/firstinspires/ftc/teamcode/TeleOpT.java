package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Lift.DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.TeleOpT.State.HANGING;
import static org.firstinspires.ftc.teamcode.TeleOpT.State.IDLE;
import static org.firstinspires.ftc.teamcode.TeleOpT.State.PLACING;
import static org.firstinspires.ftc.teamcode.TeleOpT.State.RETRACTING;

import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp
public class TeleOpT extends LinearOpMode {

    /*
    Normal Mode

    - left stick = move robot
    - right stick = rotate robot
    - left bumper = start/stop placing
    - right bumper = toggle turtle mode
    - left trigger = raise lift (when arm is up)
    - right trigger = lower lift (when arm is up)
    - a = toggle both claws
    - x = toggle left claw
    - b = toggle right claw
    - dpad up = hang mode
      1) raise arm and lift for hanging
      2) raise robot onto rigging
      3) lower robot from rigging (use when match ends)
    - dpad down = cancel hanging

    Debug Mode (hold right trigger)

    - x = lower wrist
    - b = raise wrist
    - a = lower arm
    - y = raise arm
    - left bumper = lower lift
    - right bumper = raise lift
    */

    enum State { IDLE, RETRACTING, HANGING, PLACING }

    public static final String ORANGE_CIRCLE = "\uD83D\uDFE0"; // See https://unicode-explorer.com/list/geometric-shapes
    public static double TRIGGER_THRESHOLD = 0.5;
    public static final String WHITE_CIRCLE = "⚪"; // See https://unicode-explorer.com/list/geometric-shapes
    public static int HANG_POSITION = 700;
    public static final int FIRST_ROW = 1;

    private RobotHardwareC robotHardware;
    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();
    private State state = IDLE;

    // Runs the op mode.
    @Override
    public void runOpMode() throws InterruptedException {

        // Get the robot hardware.
        robotHardware = new RobotHardwareC(this);

        // Initialize the drive interface.
        robotHardware.initializeDrive();

        // Start looking for AprilTags.
        //robotHardware.startLookingForAprilTags();

        // Wait for the user to lower the lift.
        //robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        //robotHardware.waitForArmDown();

        // Initialize the robot.
        robotHardware.initializeRobot();

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Notify the user that we are waiting for start.
        log("Waiting for start...");

        // Wait for start.
        waitForStart();

        // While the op mode is active...
        while (opModeIsActive()) {

            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // Determine whether the user is debugging.
            boolean debugging = currentGamepad.right_trigger > TRIGGER_THRESHOLD;

            // Switch based on the state.
            switch (state) {

                case RETRACTING:

                    handleRetracting();

                    break;

                case HANGING:

                    handleHanging(debugging);

                    break;

                case PLACING:

                    handlePlacing(debugging);

                    break;

                case IDLE:

                    handleIdle(debugging);

                    break;

                default:

                    throw new InterruptedException("Unrecognized state.");

            }

            // Move the robot.
            robotHardware.moveRobot();

            // Set turtle mode if appropriate.
            robotHardware.setTurtleMode(currentGamepad.right_bumper && !debugging);

            // Add telemetry.
            telemetry.addData("State", state);
            telemetry.addData("Debugging", debugging);

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

        }

    }

    // Logs a message.
    private void log(String message) {

        // Show the message.
        Utilities.log(message, telemetry);

    }

    // Handles the retracting state.
    private void handleRetracting() {

        // Determine whether we are actively retracting.
        boolean isRetracting = robotHardware.isRetracting();

        // If we are actively retracting...
        if (isRetracting) {

            // Exit the method.
            return;

        }

        // Advance to the idle state.
        state = IDLE;

    }

    // Handles the idle state.
    private void handleIdle(boolean debugging) throws InterruptedException {

        // If the user is debugging...
        if(debugging) {

            // If the driver pressed left bumper...
            if(currentGamepad.left_bumper && !previousGamepad.left_bumper) {

                // Raise the wrist and close the claw so they do not catch.
                robotHardware.setWristBackdrop();
                robotHardware.closeClaw();

                // Lower the lift.
                robotHardware.setLiftPosition(DOWN_POSITION);

            }

            // If the driver pressed right bumper...
            if(currentGamepad.right_bumper && !previousGamepad.right_bumper) {

                // Raise the wrist and close the claw so they do not catch.
                robotHardware.setWristBackdrop();
                robotHardware.closeClaw();

                // Raise the lift.
                robotHardware.setLiftPosition(MAXIMUM_POSITION);

            }

            // If the driver pressed x...
            if(currentGamepad.x && !previousGamepad.x) {

                // Move the wrist to the ground position.
                robotHardware.setWristGround();

            }

            // If the driver pressed b...
            if(currentGamepad.b && !previousGamepad.b) {

                // Move the wrist to the backdrop position.
                robotHardware.setWristBackdrop();

            }

            // If the driver pressed a...
            if(currentGamepad.a && !previousGamepad.a) {

                // Raise the wrist and close the claw so they do not catch.
                robotHardware.setWristBackdrop();
                robotHardware.closeClaw();

                // Lower the arm.
                robotHardware.lowerArm();

            }

            // If the driver pressed y...
            if(currentGamepad.y && !previousGamepad.y) {

                // Raise the wrist and close the claw so they do not catch.
                robotHardware.setWristBackdrop();
                robotHardware.closeClaw();

                // Raise the arm.
                robotHardware.raiseArm();

            }

        }

        // Otherwise (if the user is not debugging)...
        else {

            // If the driver pressed x...
            if(currentGamepad.x && !previousGamepad.x) {

                // Toggle the left claw.
                robotHardware.toggleLeftClaw(true);

            }

            // If the driver pressed b...
            if(currentGamepad.b && !previousGamepad.b) {

                // Toggle the right claw.
                robotHardware.toggleRightClaw(true);

            }

            // If the driver pressed a...
            if(currentGamepad.a && !previousGamepad.a) {

                // Toggle the claws.
                robotHardware.toggleClaws(true);

            }

            // If the driver pressed dpad up...
            if(currentGamepad.dpad_up && !previousGamepad.dpad_up) {

                // Start hanging mode.
                robotHardware.startHanging();

                // Advance to the hanging state.
                state = HANGING;

            }

            // If the driver pressed the left bumper...
            if(currentGamepad.left_bumper && !previousGamepad.left_bumper) {

                // Start placing mode.
                robotHardware.startPlacing();

                // Advance to the placing state.
                state = PLACING;

            }

        }

    }

    // Handles the hanging state.
    private void handleHanging(boolean debugging) {

        // If the driver pressed dpad down...
        if(currentGamepad.dpad_down && !previousGamepad.dpad_down && !debugging) {

            // Stop hanging.
            robotHardware.stopHanging();

            // Start retracting.
            robotHardware.startRetracting();

            // Advance to the retracting state.
            state = RETRACTING;

            // Exit the method.
            return;

        }

        // Determine whether we are actively hanging.
        boolean isHanging = robotHardware.isHanging();

        // If we are actively hanging...
        if (isHanging) {

            // Exit the method.
            return;

        }

        // Advance to the idle state.
        state = IDLE;

    }

    // Handles the placing state.
    private void handlePlacing(boolean debugging) {

        // If the driver pressed the left bumper...
        if(currentGamepad.left_bumper && !previousGamepad.left_bumper && !debugging) {

            // Stop placing.
            robotHardware.stopPlacing();

            // Start retracting.
            robotHardware.startRetracting();

            // Advance to the retracting state.
            state = RETRACTING;

            // Exit the method.
            return;

        }

        // Determine whether we are actively placing.
        boolean isPlacing = robotHardware.isPlacing();

        // If we are actively placing...
        if (isPlacing) {

            // Exit the method.
            return;

        }

        // Advance to the idle state.
        state = IDLE;

    }

}
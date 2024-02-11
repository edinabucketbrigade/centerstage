package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoF.FIRST_ROW;
import static org.firstinspires.ftc.teamcode.AutoF.getLeftColumn;
import static org.firstinspires.ftc.teamcode.AutoF.lastRanAutonomous;
import static org.firstinspires.ftc.teamcode.HeatSeekC.MAXIMUM_ROW;
import static org.firstinspires.ftc.teamcode.Lift.DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.MAXIMUM_POSITION;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.MINIMUM_COLUMN;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.MINIMUM_ROW;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.getMaximumColumn;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.isEven;
import static org.firstinspires.ftc.teamcode.TeleOpT.State.HANGING;
import static org.firstinspires.ftc.teamcode.TeleOpT.State.HEAT_SEEKING;
import static org.firstinspires.ftc.teamcode.TeleOpT.State.IDLE;
import static org.firstinspires.ftc.teamcode.TeleOpT.State.PLACING;
import static org.firstinspires.ftc.teamcode.TeleOpT.State.RETRACTING;

import static bucketbrigade.casperlibrary.TeamPropLocation.LEFT;
import static bucketbrigade.casperlibrary.TeamPropLocation.MIDDLE;
import static bucketbrigade.casperlibrary.TeamPropLocation.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    - left trigger = lower lift (when arm is up)
    - right trigger = raise lift (when arm is up)
    - a = close both claws
    - y = open both claws
    - x = toggle left claw
    - b = toggle right claw
    - dpad up = hang mode
      1) raise arm and lift for hanging
      2) raise robot onto rigging
      3) lower robot from rigging (use when match ends)
    - dpad down = cancel hanging

    Debug Mode (hold right trigger)

    - dpad left = start heat seeking left
    - dpad down = start heat seeking middle
    - dpad right = start heat seeking right
    - dpad up = cancel heat seeking
    - x = lower wrist
    - b = raise wrist
    - a = lower arm
    - y = raise arm
    - left bumper = lower lift
    - right bumper = raise lift
    */

    enum State { IDLE, HEAT_SEEKING, RETRACTING, HANGING, PLACING }

    public static final String ORANGE_CIRCLE = "\uD83D\uDFE0"; // See https://unicode-explorer.com/list/geometric-shapes
    public static double TRIGGER_THRESHOLD = 0.5;
    public static final String WHITE_CIRCLE = "⚪"; // See https://unicode-explorer.com/list/geometric-shapes
    public static int HANG_POSITION = 700;

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
        robotHardware.startLookingForAprilTags();

        // If we last ran an autonomous op mode...
        if(lastRanAutonomous) {

            // Get the robot's pose at the end of the autonomous op mode.
            Pose2d currentPose = AutoF.currentPose;

            // Set the robot's pose.
            robotHardware.setPose(currentPose);

        }

        // Otherwise (if we last ran a tele op mode)...
        else {

            // Clear the red alliance value.
            AutoF.redAlliance = null;

        }

        // Remember that we last ran a tele op mode.
        lastRanAutonomous = false;

        // Wait for menu selection.
        waitForMenuSelection();

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

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

                case HEAT_SEEKING:

                    handleHeatSeeking(debugging);

                    break;

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

            // Determine whether we are heat seeking.
            boolean isHeatSeeking = robotHardware.isHeatSeeking();

            // If we are not heat seeking...
            if(!isHeatSeeking) {

                // Move the robot.
                robotHardware.moveRobot();

            }

            // If the driver pressed right bumper...
            if(currentGamepad.right_bumper && !previousGamepad.right_bumper && !debugging) {

                // Toggle turtle mode.
                robotHardware.toggleTurtleMode();

            }

            // Add telemetry.
            telemetry.addData("Red Alliance", AutoF.redAlliance);
            telemetry.addData("State", state);
            telemetry.addData("Debugging", debugging);

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

        }

    }

    // Gets a hex display.
    private static String getHexDisplay(int leftColumn, int leftRow, int rightColumn, int rightRow) {

        // Previous symbols: 〇 Ⓛ Ⓡ

        // Initialize the output.
        String output = "\n";

        // For each row...
        for (int row = MAXIMUM_ROW; row >= MINIMUM_ROW; row--) {

            // If the row is odd...
            if (!isEven(row)) {

                // Append a space.
                output += "  ";

            }

            // Get the row's column count.
            int maximumColumn = getMaximumColumn(row);

            // For each column...
            for (int column = MINIMUM_COLUMN; column <= maximumColumn; column++) {

                // Append the appropriate symbol.
                if (column == leftColumn && leftRow == row) {
                    output += ORANGE_CIRCLE;
                } else if (column == rightColumn && rightRow == row) {
                    output += ORANGE_CIRCLE;
                } else {
                    output += WHITE_CIRCLE;
                }

            }

            // If this is not the bottom row...
            if(row > MINIMUM_ROW) {

                // Append a newline.
                output += "\n";

            }

        }

        // Return the output.
        return output;

    }

    // Waits for menu selection.
    private void waitForMenuSelection() throws InterruptedException {

        // While the op mode is active...
        while (!isStopRequested()) {

            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // If the user has not selected an alliance...
            if (AutoF.redAlliance == null) {
                telemetry.addData("Alliance", "X = blue, B = red");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    AutoF.redAlliance = false;
                }
                if (currentGamepad.b && !previousGamepad.b) {
                    AutoF.redAlliance = true;
                }
            }

            // Otherwise (if the user finished making menu selections)...
            else {

                // Stop prompting the user for inputs.
                break;

            }

        }

    }

    // Logs a message.
    private void log(String message) {

        // Show the message.
        Utilities.log(message, telemetry);

    }

    // Handles the heat seeking state.
    private void handleHeatSeeking(boolean debugging) {

        // If the driver pressed dpad up...
        if(currentGamepad.dpad_up && !previousGamepad.dpad_up && debugging) {

            // Stop heat seeking.
            robotHardware.stopHeatSeeking();

            // Start retracting.
            robotHardware.startRetracting();

            // Advance to the retracting state.
            state = RETRACTING;

            // Exit the method.
            return;

        }

        // Determine whether we are actively heat seeking.
        boolean isHeatSeeking = robotHardware.isHeatSeeking();

        // If we are actively heat seeking...
        if (isHeatSeeking) {

            // Exit the method.
            return;

        }

        // Start retracting.
        robotHardware.startRetracting();

        // Advance to the retracting state.
        state = RETRACTING;

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

                // Close the claw so it does not catch when lowering the lift.
                robotHardware.closeClaw();

                // Lower the lift.
                robotHardware.setLiftPosition(DOWN_POSITION);

            }

            // If the driver pressed right bumper...
            if(currentGamepad.right_bumper && !previousGamepad.right_bumper) {

                // Close the claw so it does not catch when raising the lift.
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

                // Close the claw so it does not catch when lowering the arm.
                robotHardware.closeClaw();

                // Lower the arm.
                robotHardware.lowerArm();

            }

            // If the driver pressed y...
            if(currentGamepad.y && !previousGamepad.y) {

                // Close the claw so it does not catch when raising the arm.
                robotHardware.closeClaw();

                // Raise the arm.
                robotHardware.raiseArm();

            }

            // Determine whether the robot is localized.
            boolean localized = robotHardware.isLocalized();

            // If the robot is localized...
            if(localized) {

                // If the driver pressed dpad left...
                if(currentGamepad.dpad_left && !previousGamepad.dpad_left) {

                    // Get the left pixel location.
                    int leftColumn = getLeftColumn(LEFT);

                    // Start heat seeking.
                    robotHardware.startHeatSeeking(leftColumn, FIRST_ROW, AutoF.redAlliance);

                    // Advance to the heat seeking state.
                    state = HEAT_SEEKING;

                }

                // If the driver pressed dpad down...
                if(currentGamepad.dpad_down && !previousGamepad.dpad_down) {

                    // Get the middle pixel location.
                    int leftColumn = getLeftColumn(MIDDLE);

                    // Start heat seeking.
                    robotHardware.startHeatSeeking(leftColumn, FIRST_ROW, AutoF.redAlliance);

                    // Advance to the heat seeking state.
                    state = HEAT_SEEKING;

                }

                // If the driver pressed dpad right...
                if(currentGamepad.dpad_right && !previousGamepad.dpad_right) {

                    // Get the right pixel location.
                    int leftColumn = getLeftColumn(RIGHT);

                    // Start heat seeking.
                    robotHardware.startHeatSeeking(leftColumn, FIRST_ROW, AutoF.redAlliance);

                    // Advance to the heat seeking state.
                    state = HEAT_SEEKING;

                }

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

                // Close the claws.
                robotHardware.closeClaw();

            }

            // If the driver pressed y...
            if(currentGamepad.y && !previousGamepad.y) {

                // Open the claws.
                robotHardware.openClaw(true);

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
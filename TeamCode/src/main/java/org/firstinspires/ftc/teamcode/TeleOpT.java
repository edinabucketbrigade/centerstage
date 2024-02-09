package org.firstinspires.ftc.teamcode;

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
import static org.firstinspires.ftc.teamcode.TeleOpT.State.RETRACTING;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp
public class TeleOpT extends LinearOpMode {

    /*
    Gamepad 1: Robot Driver

    - left stick = move robot
    - right stick = rotate robot
    - right bumper = hold for turtle mode
    - x = toggle left claw
    - b = toggle right claw
    - a = toggle both claws
    - dpad up = hang mode
      1) raise arm and lift for hanging
      2) raise robot onto rigging
      3) lower robot from rigging (use when match ends)
    - y = cancel hanging

    Gamepad 2: Pixel Driver

    Normal Mode

    - dpad = move pixels
    - a = start heat seek
    - y = cancel heat seek

    Debug Mode (hold right trigger)

    - x = lower wrist
    - b = raise wrist
    - a = lower arm
    - y = raise arm
    - dpad down = lower lift
    - dpad up = raise lift
     */

    enum State { IDLE, HEAT_SEEKING, RETRACTING, HANGING }

    public static final String ORANGE_CIRCLE = "\uD83D\uDFE0"; // See https://unicode-explorer.com/list/geometric-shapes
    public static double TRIGGER_THRESHOLD = 0.5;
    public static final String WHITE_CIRCLE = "⚪"; // See https://unicode-explorer.com/list/geometric-shapes
    public static int HANG_POSITION = 790;

    private RobotHardwareC robotHardware;
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();

    private State state = IDLE;
    private int leftColumn = MINIMUM_COLUMN;
    private int leftRow = MINIMUM_ROW;

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
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // Determine whether the user is debugging.
            boolean debugging = currentGamepad2.right_trigger > TRIGGER_THRESHOLD;

            // Switch based on the state.
            switch (state) {

                case HEAT_SEEKING:

                    handleHeatSeeking(debugging);

                    break;

                case RETRACTING:

                    handleRetracting();

                    break;

                case HANGING:

                    handleHanging();

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

            // Set turtle mode.
            robotHardware.setTurtleMode(currentGamepad1.right_bumper);

            // Compute the right column and row.
            int rightColumn = leftColumn + 1;
            int rightRow = leftRow;

            // Get the hex display.
            String output = getHexDisplay(leftColumn,leftRow,rightColumn,rightRow);

            // Add telemetry.
            telemetry.addData("Pixel Placement", output);
            telemetry.addData("Left Pixel", "Column = %d, Row = %d", leftColumn, leftRow);
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
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // If the user has not selected an alliance...
            if (AutoF.redAlliance == null) {
                telemetry.addData("Alliance", "X = blue, B = red");
                telemetry.update();
                if (currentGamepad1.x && !previousGamepad1.x) {
                    AutoF.redAlliance = false;
                }
                if (currentGamepad1.b && !previousGamepad1.b) {
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

        // If the pixel driver pressed y...
        if(currentGamepad2.y && !previousGamepad2.y && !debugging) {

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

        // If the robot driver pressed x...
        if(currentGamepad1.x && !previousGamepad1.x) {

            // Toggle the left claw.
            robotHardware.toggleLeftClaw();

        }

        // If the robot driver pressed b...
        if(currentGamepad1.b && !previousGamepad1.b) {

            // Toggle the right claw.
            robotHardware.toggleRightClaw();

        }

        // If the robot driver pressed a...
        if(currentGamepad1.a && !previousGamepad1.a) {

            // Toggle the claws.
            robotHardware.toggleClaws();

        }

        // If the robot driver pressed dpad up...
        if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {

            // Start hanging mode.
            robotHardware.startHanging();

            // Advance to the hanging state.
            state = HANGING;

        }

        // If the user is debugging...
        if(debugging) {

            // If the pixel driver pressed dpad down...
            if(currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {

                // Close the claw so it does not catch when lowering the lift.
                robotHardware.closeClaw();

                // Lower the lift.
                robotHardware.setLiftPosition(DOWN_POSITION);

            }

            // If the pixel driver pressed dpad up...
            if(currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {

                // Close the claw so it does not catch when raising the lift.
                robotHardware.closeClaw();

                // Raise the lift.
                robotHardware.setLiftPosition(MAXIMUM_POSITION);

            }

            // If the pixel driver pressed x...
            if(currentGamepad2.x && !previousGamepad2.x) {

                // Move the wrist to the ground position.
                robotHardware.setWristGround();

            }

            // If the pixel driver pressed b...
            if(currentGamepad2.b && !previousGamepad2.b) {

                // Move the wrist to the backdrop position.
                robotHardware.setWristBackdrop();

            }

            // If the pixel driver pressed a...
            if(currentGamepad2.a && !previousGamepad2.a) {

                // Close the claw so it does not catch when lowering the arm.
                robotHardware.closeClaw();

                // Lower the arm.
                robotHardware.lowerArm();

            }

            // If the pixel driver pressed y...
            if(currentGamepad2.y && !previousGamepad2.y) {

                // Close the claw so it does not catch when raising the arm.
                robotHardware.closeClaw();

                // Raise the arm.
                robotHardware.raiseArm();

            }

        }

        // Otherwise (if the user is not debugging)...
        else {

            // If the pixel driver pressed dpad right...
            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {

                // Increment the left column.
                int maximumColumn = getMaximumColumn(leftRow);
                leftColumn = Math.min(leftColumn + 1, maximumColumn - 1);

            }

            // If the pixel driver pressed dpad left...
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {

                // Decrement the left column.
                leftColumn = Math.max(leftColumn - 1, MINIMUM_COLUMN);

            }

            // If the pixel driver pressed dpad up...
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {

                // Increment the left row.
                leftRow = Math.min(leftRow + 1, MAXIMUM_ROW);

                // Update the left column if needed.
                int maximumColumn = getMaximumColumn(leftRow);
                if (leftColumn >= maximumColumn) {
                    leftColumn = maximumColumn - 1;
                }

            }

            // If the pixel driver pressed dpad down...
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {

                // Decrement the left row.
                leftRow = Math.max(leftRow - 1, MINIMUM_ROW);

                // Update the left column if needed.
                int maximumColumn = getMaximumColumn(leftRow);
                if (leftColumn >= maximumColumn) {
                    leftColumn = maximumColumn - 1;
                }

            }

            // Determine whether the robot is localized.
            boolean localized = robotHardware.isLocalized();

            // If the robot is localized and the pixel driver pressed a...
            if(localized && currentGamepad2.a && !previousGamepad2.a) {

                // Start heat seeking.
                robotHardware.startHeatSeeking(leftColumn, leftRow, AutoF.redAlliance);

                // Advance to the heat seeking state.
                state = HEAT_SEEKING;

            }

        }

    }

    // Handles the hanging state.
    private void handleHanging() {

        // If the robot driver pressed y...
        if(currentGamepad1.y && !previousGamepad1.y) {

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

}
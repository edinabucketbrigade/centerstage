package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoF.lastRanAutonomous;
import static org.firstinspires.ftc.teamcode.HeatSeekC.MAXIMUM_ROW;
import static org.firstinspires.ftc.teamcode.Lift.MAXIMUM_POSITION;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.MINIMUM_COLUMN;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.MINIMUM_ROW;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.getMaximumColumn;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.isEven;
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

    Gamepad 2: Pixel Driver

    Normal Mode

    - dpad = move pixels
    - a = start heat seek
    - y = stop heat seek
    - x = retract

    Debug Mode (hold right trigger)

    - x = lower wrist
    - b = raise wrist
    - a = lower arm
    - y = raise arm
    - dpad down = lower lift
    - dpad up = raise lift
     */

    enum State { IDLE, HEAT_SEEKING, RETRACTING }

    public static final String ORANGE_CIRCLE = "\uD83D\uDFE0"; // See https://unicode-explorer.com/list/geometric-shapes
    public static double TRIGGER_THRESHOLD = 0.5;
    public static final String WHITE_CIRCLE = "⚪"; // See https://unicode-explorer.com/list/geometric-shapes

    private RobotHardwareC robotHardware;
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();

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

        // Initialize left pixel's column and row.
        int leftColumn = MINIMUM_COLUMN;
        int leftRow = MINIMUM_ROW;

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

            // Determine whether the robot is localized.
            boolean localized = robotHardware.isLocalized();

            switch (state) {
                case HEAT_SEEKING:

                    // Determine whether we are heat seeking.
                    boolean isHeatSeeking = robotHardware.isHeatSeeking();

                    if (isHeatSeeking) {
                        break;
                    }

                    if (currentGamepad2.x && !previousGamepad2.x && !debugging) {
                        // Start retracting.
                        robotHardware.startRetracting();

                        state = RETRACTING;
                    }

                    break;
                case RETRACTING:

                    // Determine whether we are retracting.
                    boolean isRetracting = robotHardware.isRetracting();

                    if (isRetracting) {
                        break;
                    }

                    state = IDLE;

                    break;
                case IDLE:

                    // If the robot is localized and the pixel driver pressed a...
                    if(localized && currentGamepad2.a && !previousGamepad2.a && !debugging) {

                        // Start heat seeking.
                        robotHardware.startHeatSeeking(leftColumn, leftRow, AutoF.redAlliance);

                        state = HEAT_SEEKING;
                    }

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

            // Set turtle mode.
            robotHardware.setTurtleMode(currentGamepad1.right_bumper);

            // If the user is debugging...
            if(debugging) {

                // If the pixel driver pressed dpad down...
                if(currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {

                    // Close the claw so it does not catch when lowering the lift.
                    robotHardware.closeClaw();

                    // Lower the lift.
                    robotHardware.lowerLift();

                }

                // If the pixel driver pressed dpad up...
                if(currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {

                    // Close the claw so it does not catch when raising the lift.
                    robotHardware.closeClaw();

                    // Raise the lift.
                    robotHardware.raiseLift(MAXIMUM_POSITION);

                }

                // If the pixel driver pressed x...
                if(currentGamepad2.x && !previousGamepad2.x) {

                    // Lower the wrist.
                    robotHardware.lowerWrist();

                }

                // If the pixel driver pressed b...
                if(currentGamepad2.b && !previousGamepad2.b) {

                    // Toggle the wrist.
                    robotHardware.raiseWrist();

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

                // If the pixel driver pressed y...
                if(currentGamepad2.y && !previousGamepad2.y) {

                    // RM: I am disabling this for now because it does not a) cancel the trajectory
                    // or b) return robot to a neutral position and

                    // Stop heat seeking.
                    //robotHardware.stopHeatSeeking();

                }

            }

            // Compute the right column and row.
            int rightColumn = leftColumn + 1;
            int rightRow = leftRow;

            // Get the hex display.
            String output = getHexDisplay(leftColumn,leftRow,rightColumn,rightRow);

            // Add telemetry.
            telemetry.addData("Pixel Placement", output);
            telemetry.addData("Left Pixel", "Column = %d, Row = %d", leftColumn, leftRow);
            telemetry.addData("Red Alliance", AutoF.redAlliance);
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

}
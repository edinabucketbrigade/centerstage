package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardwareB.MAXIMUM_ROW;
import static org.firstinspires.ftc.teamcode.RobotHardwareB.MINIMUM_COLUMN;
import static org.firstinspires.ftc.teamcode.RobotHardwareB.MINIMUM_ROW;
import static org.firstinspires.ftc.teamcode.RobotHardwareB.getMaximumColumn;
import static org.firstinspires.ftc.teamcode.RobotHardwareB.isEven;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp
public class TeleOpR extends LinearOpMode {

    /*
    Gamepad 1: Robot Driver

    - left stick = move robot
    - right stick = rotate robot
    - left trigger = roller intake
    - right trigger = roller eject
    - x = toggle left claw
    - b = toggle right claw
    - a = toggle both claws
    - y = hold for turtle mode
    - dpad up = raise lift
    - dpad down = lower lift

    Gamepad 2: Pixel Driver

    - x = move pixels left
    - b = move pixels right
    - a = move pixels down
    - y = move pixels up
    - dpad down = start heat seek
    - dpad up = stop heat seek
     */

    public static double TRIGGER_THRESHOLD = 0.5;
    public static String WHITE_CIRCLE = "⚪"; // See https://unicode-explorer.com/list/geometric-shapes
    public static String ORANGE_CIRCLE = "\uD83D\uDFE0"; // See https://unicode-explorer.com/list/geometric-shapes
    public static int HANG_LIFT_POSITION = 3000;

    private boolean heatSeeking = false;
    private RobotHardwareB robotHardware;
    // Initialize gamepads.
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();
    private Boolean redAlliance = null;

    // Runs the op mode.
    @Override
    public void runOpMode() throws InterruptedException {

        // Get the robot hardware.
        robotHardware = new RobotHardwareB(this);

        // Start looking for AprilTags.
        robotHardware.startLookingForAprilTags();

        // Initialize left pixel's column and row.
        int leftColumn = MINIMUM_COLUMN;
        int leftRow = MINIMUM_ROW;

        // Wait for menu selection.
        waitForMenuSelection();

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Initialize the robot.
        robotHardware.initializeRobot();

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Notify the user that we are waiting for start.
        robotHardware.log("Waiting for start...");

        // Wait for start.
        waitForStart();

        // While the op mode is active...
        while (opModeIsActive()) {

            // Update the gamepads.
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // If we are not heat seeking...
            if(!heatSeeking) {

                // Move the robot.
                robotHardware.moveRobot();

            }

            // If the pixel driver pressed dpad down...
            if(currentGamepad2.dpad_down) {

                // Start heat seeking.
                heatSeeking = true;

            }

            // If the pixel drive pressed dpad up...
            if (currentGamepad2.dpad_up) {

                // Stop heat seeking.
                heatSeeking = false;

            }

            // Determine whether the robot is localized.
            boolean localized = robotHardware.isLocalized();

            // If we are heat seeking and we know the robot's location...
            if (heatSeeking && localized) {

                // Heat seek.
                robotHardware.heatSeek(leftColumn, leftRow, redAlliance);

                // Remember that we are done heat seeking.
                heatSeeking = false;

            }

            if (currentGamepad1.back && !previousGamepad1.back) {
                robotHardware.placePixelsOnBackdrop(leftRow);
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

            // If the robot driver is holding the left trigger...
            if (currentGamepad1.left_trigger > TRIGGER_THRESHOLD) {

                // Power the roller to intake.
                robotHardware.intakeRoller();

            }

            // Otherwise, if the robot driver is hold the right trigger...
            else if (currentGamepad1.right_trigger > TRIGGER_THRESHOLD) {

                // Power the roller to eject.
                robotHardware.ejectRoller();

            }

            // Otherwise (if the robot driver is not holding a trigger)...
            else {

                // Stop the intake.
                robotHardware.stopRoller();

            }

            // If the robot driver pressed dpad down...
            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {

                // Lower the lift.
                robotHardware.lowerLift();

            }

            // If the robot driver pressed dpad up...
            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {

                // Raise the lift.
                robotHardware.raiseLift(HANG_LIFT_POSITION);

            }

            // If the robot driver is holding y...
            if(currentGamepad1.y) {

                // Enable turtle mode.
                robotHardware.setTurtleMode(true);

            }

            // Otherwise (if the robot driver is not holding y)...
            else {

                // Disable turtle mode.
                robotHardware.setTurtleMode(false);

            }

            // If the pixel driver pressed b...
            if (currentGamepad2.b && !previousGamepad2.b) {

                // Increment the left column.
                int maximumColumn = getMaximumColumn(leftRow);
                leftColumn = Math.min(leftColumn + 1, maximumColumn - 1);

            }

            // If the pixel driver pressed x...
            if (currentGamepad2.x && !previousGamepad2.x) {

                // Decrement the left column.
                leftColumn = Math.max(leftColumn - 1, MINIMUM_COLUMN);

            }

            // If the pixel driver pressed y...
            if (currentGamepad2.y && !previousGamepad2.y){

                // Increment the left row.
                leftRow = Math.min(leftRow + 1, MAXIMUM_ROW);

                // Update the left column if needed.
                int maximumColumn = getMaximumColumn(leftRow);
                if(leftColumn >= maximumColumn) {
                    leftColumn = maximumColumn - 1;
                }

            }

            // If the pixel driver pressed a...
            if (currentGamepad2.a && !previousGamepad2.a){

                // Decrement the left row.
                leftRow = Math.max(leftRow - 1, MINIMUM_ROW);

                // Update the left column if needed.
                int maximumColumn = getMaximumColumn(leftRow);
                if(leftColumn >= maximumColumn) {
                    leftColumn = maximumColumn - 1;
                }

            }

            // Compute the right column and row.
            int rightColumn = leftColumn + 1;
            int rightRow = leftRow;

            // Get the hex display.
            String output = getHexDisplay(leftColumn,leftRow,rightColumn,rightRow);

            // Add telemetry.
            telemetry.addData("Pixel Placement", output);
            telemetry.addData("Left Column", leftColumn);
            telemetry.addData("Left Row", leftRow);
            telemetry.addData("Right Column", rightColumn);
            telemetry.addData("Right Row", rightRow);

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
            if (redAlliance == null) {
                telemetry.addData("Alliance", "X = blue, B = red");
                telemetry.update();
                if (currentGamepad1.x && !previousGamepad1.x) {
                    redAlliance = false;
                }
                if (currentGamepad1.b && !previousGamepad1.b) {
                    redAlliance = true;
                }
            }

            // Otherwise (if the user finished making menu selections)...
            else {

                // Stop prompting the user for inputs.
                break;

            }

        }

    }

}
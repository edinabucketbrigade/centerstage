package bucketbrigade.casperlibrary;

import static bucketbrigade.casperlibrary.Action.mirrorActions;
import static bucketbrigade.casperlibrary.Action.mirrorLocation;
import static bucketbrigade.casperlibrary.Action.mirrorPose;
import static bucketbrigade.casperlibrary.TeamPropLocation.LEFT;
import static bucketbrigade.casperlibrary.TeamPropLocation.MIDDLE;
import static bucketbrigade.casperlibrary.TeamPropLocation.RIGHT;

import java.util.ArrayList;
import java.util.List;

public class RobotRoutes {

    public static final double BACKDROP_TARGET_X = 40;
    public static final int FIRST_ROW_LIFT_POSITION = 0;
    public static final int LIFT_INCREMENT = 200;
    public static final int MAXIMUM_COLUMN_EVEN_ROW = 7;
    public static final int MAXIMUM_COLUMN_ODD_ROW = 6;
    public static final int MAXIMUM_POSITION = 1700;
    public static final int MAXIMUM_ROW = (int)Math.floor((MAXIMUM_POSITION - FIRST_ROW_LIFT_POSITION) / LIFT_INCREMENT);
    public static final int MINIMUM_COLUMN = 1;
    public static final int MINIMUM_ROW = 1;
    public static double PIXEL_WIDTH = 3.2;
    public static final double PLACE_TARGET_X = 43.5;
    public static double TARGET_BLUE_Y = 43;
    public static double TARGET_RED_Y = -28;
    public static final int WHITE_PIXEL_LEFT_COLUMN = 4;
    public static final int WHITE_PIXEL_ROW = 3;
    public static final int YELLOW_PIXEL_ROW = 1;

    // Gets a start pose.
    public static RobotPose getStartPose(boolean redAlliance, boolean startClose) {

        RobotPose startPose;
        if(startClose) {
            startPose = new RobotPose(12, 61, Math.toRadians(90));
        }
        else {
            startPose = new RobotPose(-36, 61, Math.toRadians(90));
        }
        if(redAlliance) mirrorPose(startPose);
        return startPose;

    }

    // Drives to a spike mark.
    public static List<Action> driveToSpikeMark(boolean redAlliance, boolean startClose, TeamPropLocation inputLocation) {

        TeamPropLocation outputLocation = redAlliance ? mirrorLocation(inputLocation) : inputLocation;
        List<Action> actions = new ArrayList<>();
        if (startClose) {
            actions.add(new SetReversedAction(true));
            if (outputLocation == LEFT) {
                actions.add(new SplineToAction(32.5, 30, Math.toRadians(0)));
            } else if (outputLocation == MIDDLE) {
                actions.add(new SplineToAction(25, 27, Math.toRadians(0)));
            } else {
                actions.add(new SplineToLinearHeadingAction(11, 33, Math.toRadians(180), Math.toRadians(180)));
            }
        } else {
            if (outputLocation == LEFT) {
                actions.add(new SetReversedAction(true));
                actions.add(new SplineToLinearHeadingAction(-35, 27, Math.toRadians(0), Math.toRadians(0)));
            } else if (outputLocation == MIDDLE) {
                actions.add(new LineToLinearHeadingAction(-36, 14, Math.toRadians(90)));
            } else {
                actions.add(new LineToAction(-46.5, 19));
            }
        }
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Drives to the backdrop.
    public static List<Action> driveToBackdrop(boolean redAlliance, boolean startClose, TeamPropLocation inputLocation) throws InterruptedException {

        double targetY = getWhitePixelTargetY();
        TeamPropLocation outputLocation = redAlliance ? mirrorLocation(inputLocation) : inputLocation;
        double targetHeading = Math.toRadians(180);
        List<Action> actions = new ArrayList<>();
        if (startClose) {
            actions.add(new LineToLinearHeadingAction(BACKDROP_TARGET_X, targetY, targetHeading));
        } else {
            actions.add(new SetReversedAction(true));
            if(outputLocation == LEFT) {
                actions.add(new SplineToAction(-46, 9, Math.toRadians(0)));
            }
            else if(outputLocation == MIDDLE) {
                actions.add(new SplineToAction(-33, 9, Math.toRadians(0)));
            }
            else {
                actions.add(new SplineToAction(-36, 9, Math.toRadians(0)));
            }
            actions.add(new LineToAction(30, 9));
            actions.add(new SplineToAction(BACKDROP_TARGET_X, targetY, Math.toRadians(0)));
        }
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Drives to the placement position.
    public static List<Action> driveToPlace(boolean redAlliance, TeamPropLocation inputLocation, boolean isYellowPixel) throws InterruptedException {

        TeamPropLocation outputLocation = redAlliance ? mirrorLocation(inputLocation) : inputLocation;
        double targetY = isYellowPixel ? getYellowPixelTargetY(outputLocation) : getWhitePixelTargetY();
        List<Action> actions = new ArrayList<>();
        actions.add(new LineToLinearHeadingAction(PLACE_TARGET_X, targetY, Math.toRadians(180)));
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Drives to a white pixel stack.
    public static List<Action> driveToStack(boolean redAlliance) {

        List<Action> actions = new ArrayList<>();
        actions.add(new SetTangentAction(Math.toRadians(-90)));
        actions.add(new SplineToLinearHeadingAction(20, 8, Math.toRadians(180), Math.toRadians(180)));
        actions.add(new LineToAction(-59, 8));
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Returns to the backdrop.
    public static List<Action> returnToBackdrop(boolean redAlliance) throws InterruptedException {

        double targetY = getWhitePixelTargetY();
        List<Action> actions = new ArrayList<>();
        actions.add(new LineToAction(20, 8));
        actions.add(new SplineToLinearHeadingAction(BACKDROP_TARGET_X, targetY, Math.toRadians(180), Math.toRadians(90)));
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Parks.
    public static List<Action> park(boolean redAlliance, boolean inputParkLeft) {

        boolean outputParkLeft = redAlliance ? !inputParkLeft : inputParkLeft;
        List<Action> actions = new ArrayList<>();
        if (outputParkLeft) {
            actions.add(new SetTangentAction(Math.toRadians(90)));
            actions.add(new SplineToConstantHeadingAction(44, 60, Math.toRadians(0)));
            actions.add(new SplineToConstantHeadingAction(58, 60, Math.toRadians(0)));
        } else {
            actions.add(new SetTangentAction(Math.toRadians(-90)));
            actions.add(new SplineToConstantHeadingAction(44, 12, Math.toRadians(0)));
            actions.add(new SplineToConstantHeadingAction(58, 12, Math.toRadians(0)));
        }
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    private static double getYellowPixelTargetY(TeamPropLocation location) throws InterruptedException {

        // Get the appropriate yellow pixel left column.
        int yellowPixelLeftColumn = getLeftColumn(location);

        // Get a target y coordinate.
        double targetY = getTargetY(yellowPixelLeftColumn, YELLOW_PIXEL_ROW, false);

        // Return the result.
        return targetY;

    }

    private static double getWhitePixelTargetY() throws InterruptedException {

        // Get a target y coordinate.
        double targetY = getTargetY(WHITE_PIXEL_LEFT_COLUMN, WHITE_PIXEL_ROW, false);

        // Return the result.
        return targetY;

    }

    public static int getLeftColumn(TeamPropLocation location) throws InterruptedException {
        if (location == LEFT) {
            return 1;
        } else if (location == MIDDLE) {
            return 3;
        } else if (location == RIGHT) {
            return 5;
        } else {
            throw new InterruptedException("The location is missing.");
        }
    }

    // Get a target y coordinate.
    public static double getTargetY(int leftColumn, int row, boolean redAlliance) throws InterruptedException {

        // If the row is invalid...
        if(row < MINIMUM_ROW || row > MAXIMUM_ROW) {

            // Complain.
            throw new InterruptedException("The row is invalid.");

        }

        // Get the row's column count.
        int maximumColumn = getMaximumColumn(row);

        // If the left column is invalid...
        if(leftColumn < MINIMUM_COLUMN || leftColumn > maximumColumn - 1) {

            // Complain.
            throw new InterruptedException("The left column is invalid.");

        }

        // Initialize a target y coordinate.
        double targetY = redAlliance ? TARGET_RED_Y : TARGET_BLUE_Y;

        // Determine whether this is an even row.
        boolean isEvenRow = isEven(row);

        // If this is an even row...
        if(isEvenRow) {

            // Shift the robot up by half a pixel width.
            targetY += PIXEL_WIDTH / 2;

        }

        // Shift the robot down to the appropriate column.
        targetY -= (leftColumn - 1) * PIXEL_WIDTH;

        // Return the result.
        return targetY;

    }

    // Gets the column count for a specified row.
    public static int getMaximumColumn(int row){
        if (isEven(row)){
            return MAXIMUM_COLUMN_EVEN_ROW;
        } else {
            return MAXIMUM_COLUMN_ODD_ROW;
        }
    }

    // Determines whether a number is even.
    public static boolean isEven(int value) {
        if (value % 2 == 0) {
            return true;
        } else {
            return false;
        }
    }

}
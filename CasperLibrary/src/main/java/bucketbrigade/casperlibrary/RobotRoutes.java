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

    public static final double BACKDROP_APPROACH_DISTANCE = 5;
    public static final double STACK_APPROACH_DISTANCE = 8;
    public static final double BACKDROP_SEPARATION = 36;
    public static final int LOW_FIRST_ROW_LIFT_POSITION = 0;
    public static final int HIGH_FIRST_ROW_LIFT_POSITION = 200;
    public static final int LIFT_INCREMENT = 200;
    public static final double MAXIMUM_ACCELERATION = 50;
    public static final double MAXIMUM_ANGULAR_ACCELERATION = Math.toRadians(100);
    public static final double MAXIMUM_ANGULAR_VELOCITY = Math.toRadians(100);
    public static final int MAXIMUM_COLUMN_EVEN_ROW = 7;
    public static final int MAXIMUM_COLUMN_ODD_ROW = 6;
    public static final int MAXIMUM_POSITION = 1700;
    public static final int MAXIMUM_ROW = 8;
    public static final double MAXIMUM_VELOCITY_FAST = 65;
    public static final double MAXIMUM_VELOCITY_NORMAL = 55;
    public static final double MAXIMUM_VELOCITY_SLOW = 25;
    public static final int MINIMUM_COLUMN = 1;
    public static final int MINIMUM_ROW = 1;
    public static double PIXEL_WIDTH = 3;
    public static final double TRACK_WIDTH = 14;
    public static final int WHITE_PIXEL_LEFT_COLUMN = 3;
    public static final int WHITE_PIXEL_ROW = 3;
    public static final int YELLOW_PIXEL_ROW = 1;

    // Blue defaults
    public static final double DEFAULT_BLUE_PLACE_BACKDROP_X = 47;
    public static final double DEFAULT_BLUE_PLACE_BACKDROP_Y = 41.5;
    public static final double DEFAULT_BLUE_GRAB_STACK_X = -60;
    public static final double DEFAULT_BLUE_GRAB_STACK_Y = 7.5;

    // Red defaults
    public static final double DEFAULT_RED_PLACE_BACKDROP_X = DEFAULT_BLUE_PLACE_BACKDROP_X;
    public static final double DEFAULT_RED_PLACE_BACKDROP_Y = 40.5;
    public static final double DEFAULT_RED_GRAB_STACK_X = DEFAULT_BLUE_GRAB_STACK_X;
    public static final double DEFAULT_RED_GRAB_STACK_Y = 9;

    // Gets a start pose.
    public static RobotPose getStartPose(boolean redAlliance, boolean startClose) {

        RobotPose startPose;
        if(startClose) {
            startPose = new RobotPose(12, 62, Math.toRadians(90));
        }
        else {
            startPose = new RobotPose(-36, 62, Math.toRadians(90));
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

    // Drives to the backdrop approach position.
    public static List<Action> driveToBackdropApproach(boolean redAlliance, boolean startClose, TeamPropLocation inputLocation, double placeBackdropX, double placeBackdropY, double grabStackY) throws InterruptedException {

        TeamPropLocation outputLocation = redAlliance ? mirrorLocation(inputLocation) : inputLocation;
        double targetY = getYellowPixelTargetY(outputLocation, placeBackdropY);
        double targetHeading = Math.toRadians(180);
        List<Action> actions = new ArrayList<>();
        double approachBackdropX = getApproachBackdropX(placeBackdropX);
        if (startClose) {
            actions.add(new LineToLinearHeadingAction(approachBackdropX, targetY, targetHeading));
        } else {
            actions.add(new SetReversedAction(true));
            if(outputLocation == LEFT) {
                actions.add(new SplineToAction(-46, grabStackY, Math.toRadians(0)));
            }
            else if(outputLocation == MIDDLE) {
                actions.add(new SplineToAction(-33, grabStackY, Math.toRadians(0)));
            }
            else {
                actions.add(new SplineToAction(-36, grabStackY, Math.toRadians(0)));
            }
            actions.add(new LineToAction(30, grabStackY));
            actions.add(new SplineToAction(approachBackdropX, targetY, Math.toRadians(0)));
        }
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Drives to the backdrop place position.
    public static List<Action> driveToBackdropPlace(boolean redAlliance, TeamPropLocation inputLocation, boolean isYellowPixel, double placeBackdropX, double placeBackdropY) throws InterruptedException {

        TeamPropLocation outputLocation = redAlliance ? mirrorLocation(inputLocation) : inputLocation;
        double targetY = isYellowPixel ? getYellowPixelTargetY(outputLocation, placeBackdropY) : getWhitePixelTargetY(placeBackdropY);
        List<Action> actions = new ArrayList<>();
        actions.add(new LineToLinearHeadingAction(placeBackdropX, targetY, Math.toRadians(180)));
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Drives to the stack approach position.
    public static List<Action> driveToStackApproach(boolean redAlliance, double grabStackX, double grabStackY) {

        List<Action> actions = new ArrayList<>();

        double approachStackX = getApproachStackX(grabStackX);

        actions.add(new TurnAction(Math.toRadians(90)));
        actions.add(new SetTangentAction(Math.toRadians(-90)));

        // Method A
        actions.add(new SplineToLinearHeadingAction(20, grabStackY, Math.toRadians(180), Math.toRadians(180)));
        actions.add(new SetTangentAction(Math.toRadians(180)));

        // Method B
        //actions.add(new SplineToAction(20, grabStackY, Math.toRadians(180)));
        
        actions.add(new SplineToAction(approachStackX, grabStackY, Math.toRadians(180)));
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Drives to the stack grab position.
    public static List<Action> driveToStackGrab(boolean redAlliance, double grabStackX, double grabStackY) {

        List<Action> actions = new ArrayList<>();
        actions.add(new LineToAction(grabStackX, grabStackY));
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Returns to the backdrop.
    public static List<Action> returnToBackdrop(boolean redAlliance, double placeBackdropX, double placeBackdropY, double grabStackY) throws InterruptedException {

        double targetY = getWhitePixelTargetY(placeBackdropY);
        double approachBackdropX = getApproachBackdropX(placeBackdropX);
        List<Action> actions = new ArrayList<>();
        actions.add(new LineToAction(20, grabStackY));
        actions.add(new SplineToLinearHeadingAction(approachBackdropX, targetY, Math.toRadians(180), Math.toRadians(90)));
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
            actions.add(new SplineToConstantHeadingAction(44, 8, Math.toRadians(0)));
            actions.add(new SplineToConstantHeadingAction(58, 8, Math.toRadians(0)));
        }
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    private static double getYellowPixelTargetY(TeamPropLocation location, double placeBackdropY) throws InterruptedException {

        // Get the appropriate yellow pixel left column.
        int yellowPixelLeftColumn = getLeftColumn(location);

        // Get a target y coordinate.
        double targetY = getTargetY(yellowPixelLeftColumn, YELLOW_PIXEL_ROW, false, placeBackdropY);

        // Return the result.
        return targetY;

    }

    private static double getWhitePixelTargetY(double placeBackdropY) throws InterruptedException {

        // Get a target y coordinate.
        double targetY = getTargetY(WHITE_PIXEL_LEFT_COLUMN, WHITE_PIXEL_ROW, false, placeBackdropY);

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
    public static double getTargetY(int leftColumn, int row, boolean redAlliance, double placeBackdropY) throws InterruptedException {

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

        // Initialize the target y value to the place backdrop y value.
        double targetY = placeBackdropY;

        // If we are on the red alliance.
        if(redAlliance) {

            // Update the target y value.
            targetY -= BACKDROP_SEPARATION;

        }

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

    private static double getApproachBackdropX(double placeBackdropX) {
        return placeBackdropX - BACKDROP_APPROACH_DISTANCE;
    }

    private static double getApproachStackX(double grabStackX) {
        return grabStackX + STACK_APPROACH_DISTANCE;
    }

    public static double getDefaultPlaceBackdropX(boolean redAlliance) {
        return redAlliance ? DEFAULT_RED_PLACE_BACKDROP_X : DEFAULT_BLUE_PLACE_BACKDROP_X;
    }

    public static double getDefaultPlaceBackdropY(boolean redAlliance) {
        return redAlliance ? DEFAULT_RED_PLACE_BACKDROP_Y : DEFAULT_BLUE_PLACE_BACKDROP_Y;
    }

    public static double getDefaultGrabStackX(boolean redAlliance) {
        return redAlliance ? DEFAULT_RED_GRAB_STACK_X : DEFAULT_BLUE_GRAB_STACK_X;
    }

    public static double getDefaultGrabStackY(boolean redAlliance) {
        return redAlliance ? DEFAULT_RED_GRAB_STACK_Y : DEFAULT_BLUE_GRAB_STACK_Y;
    }

    public static double getMaximumVelocityFast(boolean redAlliance, boolean startClose, TeamPropLocation location) {
        if((redAlliance && !startClose && location == RIGHT) || (!redAlliance && !startClose && location == LEFT)) {
            return MAXIMUM_VELOCITY_FAST;
        }
        else {
            return MAXIMUM_VELOCITY_NORMAL;
        }
    }

}
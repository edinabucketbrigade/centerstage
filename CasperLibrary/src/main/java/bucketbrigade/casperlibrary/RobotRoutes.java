package bucketbrigade.casperlibrary;

import static bucketbrigade.casperlibrary.Action.mirrorActions;
import static bucketbrigade.casperlibrary.Action.mirrorPose;
import static bucketbrigade.casperlibrary.TeamPropLocation.LEFT;
import static bucketbrigade.casperlibrary.TeamPropLocation.MIDDLE;
import static bucketbrigade.casperlibrary.TeamPropLocation.RIGHT;

import java.util.ArrayList;
import java.util.List;

public class RobotRoutes {

    // Gets a start pose.
    public static RobotPose getStartPose(boolean redAlliance, boolean startLeft) {

        RobotPose startPose;
        if(startLeft) {
            startPose = new RobotPose(12, 61, Math.toRadians(90));
        }
        else {
            startPose = new RobotPose(-36, 61, Math.toRadians(90));
        }
        if(redAlliance) mirrorPose(startPose);
        return startPose;

    }

    // Drives to a spike mark.
    public static List<Action> driveToSpikeMark(boolean redAlliance, boolean startLeft, TeamPropLocation location) {

        List<Action> actions = new ArrayList<>();
        if (startLeft) {
            if (location == LEFT) {
                actions.add(new SetReversedAction(true));
                actions.add(new SplineToAction(32.5, 30, Math.toRadians(0)));
            } else if (location == MIDDLE) {
                actions.add(new LineToLinearHeadingAction(12, 14, Math.toRadians(90)));
            } else {
                actions.add(new SetReversedAction(true));
                actions.add(new SplineToLinearHeadingAction(11, 33, Math.toRadians(180), Math.toRadians(180)));
            }
        } else {
            if (location == LEFT) {
                actions.add(new SetReversedAction(true));
                actions.add(new SplineToLinearHeadingAction(-35, 27, Math.toRadians(0), Math.toRadians(0)));
            } else if (location == MIDDLE) {
                actions.add(new LineToLinearHeadingAction(-36, 14, Math.toRadians(90)));
            } else {
                actions.add(new LineToAction(-46.5, 19));
            }
        }
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Drives to the backdrop.
    public static List<Action> driveToBackdrop(boolean redAlliance, boolean startLeft, TeamPropLocation location, double targetX, double targetY) {

        double targetHeading = Math.toRadians(180);
        List<Action> actions = new ArrayList<>();
        if (startLeft) {
            if (location == MIDDLE) {
                actions.add(new BackAction(10));
            }
            actions.add(new LineToLinearHeadingAction(targetX, targetY, targetHeading));
        } else {
            actions.add(new LineToAction(-36, 9));
            if (location == MIDDLE || location == RIGHT) {
                actions.add(new TurnAction(Math.toRadians(90)));
            } else {
                actions.add(new TurnAction(Math.toRadians(180)));
            }
            actions.add(new LineToAction(30, 9));
            actions.add(new SplineToAction(targetX, targetY, Math.toRadians(0)));
        }
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Drives to a white pixel stack.
    public static List<Action> driveToStack(boolean redAlliance) {

        List<Action> actions = new ArrayList<>();
        actions.add(new SetTangentAction(Math.toRadians(-90)));
        actions.add(new SplineToLinearHeadingAction(20, 9, Math.toRadians(180), Math.toRadians(180)));
        actions.add(new LineToAction(-57, 9));
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Returns to the backdrop.
    public static List<Action> returnToBackdrop(boolean redAlliance, double targetX, double targetY) {

        List<Action> actions = new ArrayList<>();
        actions.add(new LineToAction(20, 9));
        actions.add(new SplineToLinearHeadingAction(targetX, targetY, Math.toRadians(180), Math.toRadians(90)));
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

    // Parks.
    public static List<Action> park(boolean redAlliance, boolean parkLeft) {

        List<Action> actions = new ArrayList<>();
        if (parkLeft) {
            actions.add(new SetReversedAction(false));
            actions.add(new LineToAction(44, 60));
            actions.add(new LineToAction(58, 60));
        } else {
            actions.add(new SetReversedAction(false));
            actions.add(new LineToAction(44, 12));
            actions.add(new LineToAction(58, 12));
        }
        if(redAlliance) mirrorActions(actions);
        return actions;

    }

}
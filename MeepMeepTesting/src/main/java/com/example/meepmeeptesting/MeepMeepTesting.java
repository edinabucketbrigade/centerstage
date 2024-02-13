package com.example.meepmeeptesting;

import static bucketbrigade.casperlibrary.Objectives.PURPLE_YELLOW;
import static bucketbrigade.casperlibrary.Objectives.PURPLE_YELLOW_WHITE;
import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_ACCELERATION;
import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_ANGULAR_ACCELERATION;
import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_ANGULAR_VELOCITY;
import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_VELOCITY_FAST;
import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_VELOCITY_SLOW;
import static bucketbrigade.casperlibrary.RobotRoutes.TRACK_WIDTH;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToBackdropApproach;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToBackdropPlace;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToSpikeMark;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToStackApproach;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToStackGrab;
import static bucketbrigade.casperlibrary.RobotRoutes.getStartPose;
import static bucketbrigade.casperlibrary.RobotRoutes.park;
import static bucketbrigade.casperlibrary.RobotRoutes.returnToBackdrop;
import static bucketbrigade.casperlibrary.TeamPropLocation.LEFT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.List;

import bucketbrigade.casperlibrary.Action;
import bucketbrigade.casperlibrary.BackAction;
import bucketbrigade.casperlibrary.LineToAction;
import bucketbrigade.casperlibrary.LineToLinearHeadingAction;
import bucketbrigade.casperlibrary.Objectives;
import bucketbrigade.casperlibrary.RobotPose;
import bucketbrigade.casperlibrary.SetReversedAction;
import bucketbrigade.casperlibrary.SetTangentAction;
import bucketbrigade.casperlibrary.SplineToAction;
import bucketbrigade.casperlibrary.SplineToConstantHeadingAction;
import bucketbrigade.casperlibrary.SplineToLinearHeadingAction;
import bucketbrigade.casperlibrary.TeamPropLocation;
import bucketbrigade.casperlibrary.TurnAction;

public class MeepMeepTesting {

    private static final boolean RED_ALLIANCE = false;
    private static final boolean START_CLOSE = true;
    private static final TeamPropLocation LOCATION = LEFT;
    private static final boolean PARK_LEFT = true;
    private static final Objectives objectives = PURPLE_YELLOW_WHITE;

    // Runs the application.
    public static void main(String[] args) throws Exception {

        // Get a MeepMeep interface.
        MeepMeep meepMeep = new MeepMeep(800);

        // Get a robot.
        RoadRunnerBotEntity robot = getRobot(meepMeep);

        // Start the simulation.
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();

    }

    // Gets a robot.
    private static RoadRunnerBotEntity getRobot(MeepMeep meepMeep) throws Exception {

        // Get a start pose.
        RobotPose inputStartPose = getStartPose(RED_ALLIANCE, START_CLOSE);
        Pose2d outputStartPose = new Pose2d(inputStartPose.x, inputStartPose.y, inputStartPose.heading);

        // Construct a velocity constraint.
        TrajectoryVelocityConstraint velocityConstraint = new MecanumVelocityConstraint(MAXIMUM_VELOCITY_FAST, TRACK_WIDTH);

        // Construct an acceleration constraint.
        TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(MAXIMUM_ACCELERATION);

        // Construct a trajectory sequence builder.
        TrajectorySequenceBuilder trajectorySequenceBuilder = new TrajectorySequenceBuilder(outputStartPose, null, velocityConstraint, accelerationConstraint, MAXIMUM_ANGULAR_VELOCITY, MAXIMUM_ANGULAR_ACCELERATION);

        // Drive to the spike mark.
        applyActions(driveToSpikeMark(RED_ALLIANCE, START_CLOSE, LOCATION), trajectorySequenceBuilder, true);

        // Wait for a bit.
        trajectorySequenceBuilder.waitSeconds(1);

        // If we are placing the yellow pixel...
        if(objectives == PURPLE_YELLOW || objectives == PURPLE_YELLOW_WHITE) {

            // Drive to the backdrop approach position.
            applyActions(driveToBackdropApproach(RED_ALLIANCE, START_CLOSE, LOCATION), trajectorySequenceBuilder, true);

            // Wait for a bit.
            trajectorySequenceBuilder.waitSeconds(1);

            // Drive to the backdrop place position.
            applyActions(driveToBackdropPlace(RED_ALLIANCE, LOCATION, true), trajectorySequenceBuilder, false);

            // Wait for a bit.
            trajectorySequenceBuilder.waitSeconds(1);

            // If we are placing white pixels...
            if(objectives == PURPLE_YELLOW_WHITE) {

                // Drive to the stack approach position.
                applyActions(driveToStackApproach(RED_ALLIANCE), trajectorySequenceBuilder, true);

                // Wait for a bit.
                trajectorySequenceBuilder.waitSeconds(1);

                // Drive to the stack grab position.
                applyActions(driveToStackGrab(RED_ALLIANCE), trajectorySequenceBuilder, false);

                // Wait for a bit.
                trajectorySequenceBuilder.waitSeconds(1);

                // Return to the backdrop.
                applyActions(returnToBackdrop(RED_ALLIANCE), trajectorySequenceBuilder, true);

                // Wait for a bit.
                trajectorySequenceBuilder.waitSeconds(1);

                // Drive to the backdrop place position.
                applyActions(driveToBackdropPlace(RED_ALLIANCE, LOCATION, false), trajectorySequenceBuilder, false);

                // Wait for a bit.
                trajectorySequenceBuilder.waitSeconds(1);

            }

        }

        // If we should park...
        if(START_CLOSE || objectives == PURPLE_YELLOW || objectives == PURPLE_YELLOW_WHITE) {

            // Park.
            applyActions(park(RED_ALLIANCE, PARK_LEFT), trajectorySequenceBuilder, true);

        }

        // Build a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Construct a robot.
        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep).followTrajectorySequence(trajectorySequence);

        // Return the robot.
        return robot;

    }

    public static void applyActions(List<Action> actions, TrajectorySequenceBuilder trajectorySequenceBuilder, boolean fast) throws Exception {

        // Get a maximum velocity.
        double maximumVelocity = fast ? MAXIMUM_VELOCITY_FAST : MAXIMUM_VELOCITY_SLOW;

        // Construct a velocity constraint.
        TrajectoryVelocityConstraint velocityConstraint = new MecanumVelocityConstraint(maximumVelocity, TRACK_WIDTH);

        // Construct an acceleration constraint.
        TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(MAXIMUM_ACCELERATION);

        // Apply the constraints.
        trajectorySequenceBuilder.setConstraints(velocityConstraint, accelerationConstraint);

        // Apply the actions.
        for(Action inputAction : actions) {
            if(inputAction instanceof BackAction) {
                BackAction outputAction = (BackAction)inputAction;
                trajectorySequenceBuilder.back(outputAction.distance);
            }
            else if(inputAction instanceof LineToAction) {
                LineToAction outputAction = (LineToAction)inputAction;
                trajectorySequenceBuilder.lineTo(new Vector2d(outputAction.x, outputAction.y));
            }
            else if(inputAction instanceof LineToLinearHeadingAction) {
                LineToLinearHeadingAction outputAction = (LineToLinearHeadingAction)inputAction;
                trajectorySequenceBuilder.lineToLinearHeading(new Pose2d(outputAction.x, outputAction.y, outputAction.heading));
            }
            else if(inputAction instanceof SetReversedAction) {
                SetReversedAction outputAction = (SetReversedAction)inputAction;
                trajectorySequenceBuilder.setReversed(outputAction.reversed);
            }
            else if(inputAction instanceof SetTangentAction) {
                SetTangentAction outputAction = (SetTangentAction)inputAction;
                trajectorySequenceBuilder.setTangent(outputAction.tangent);
            }
            else if(inputAction instanceof SplineToAction) {
                SplineToAction outputAction = (SplineToAction)inputAction;
                trajectorySequenceBuilder.splineTo(new Vector2d(outputAction.x, outputAction.y), outputAction.heading);
            }
            else if(inputAction instanceof SplineToConstantHeadingAction) {
                SplineToConstantHeadingAction outputAction = (SplineToConstantHeadingAction)inputAction;
                trajectorySequenceBuilder.splineToConstantHeading(new Vector2d(outputAction.x, outputAction.y), outputAction.heading);
            }
            else if(inputAction instanceof SplineToLinearHeadingAction) {
                SplineToLinearHeadingAction outputAction = (SplineToLinearHeadingAction)inputAction;
                trajectorySequenceBuilder.splineToLinearHeading(new Pose2d(outputAction.x, outputAction.y, outputAction.heading), outputAction.tangent);
            }
            else if(inputAction instanceof TurnAction) {
                TurnAction outputAction = (TurnAction)inputAction;
                trajectorySequenceBuilder.turn(outputAction.angle);
            }
            else {
                throw new Exception("The action type is unrecognized.");
            }
        }

    }

}
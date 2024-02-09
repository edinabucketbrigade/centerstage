package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.TeamPropLocation.LEFT;
import static com.example.meepmeeptesting.TeamPropLocation.MIDDLE;
import static com.example.meepmeeptesting.TeamPropLocation.RIGHT;

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

public class MeepMeepTesting {

    private static final boolean RED_ALLIANCE = true;
    private static final boolean START_LEFT = true;
    private static final boolean PARK_LEFT = false;
    private static final TeamPropLocation LOCATION = RIGHT;

    public static final double MAXIMUM_VELOCITY = 60;
    public static final double MAXIMUM_ACCELERATION = 60;
    public static final double MAXIMUM_ANGULAR_VELOCITY = Math.toRadians(180);
    public static final double MAXIMUM_ANGULAR_ACCELERATION = Math.toRadians(180);
    public static final double TRACK_WIDTH = 15;
    public static final Pose2d RED_LEFT_START = new Pose2d(-36, -61, Math.toRadians(-90));
    public static final Pose2d RED_RIGHT_START = new Pose2d(12, -61, Math.toRadians(-90));
    public static final Pose2d BLUE_LEFT_START = new Pose2d(12, 61, Math.toRadians(90));
    public static final Pose2d BLUE_RIGHT_START = new Pose2d(-36, 61, Math.toRadians(90));

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
    private static RoadRunnerBotEntity getRobot(MeepMeep meepMeep) {

        // Get a start pose.
        Pose2d startPose = getStartPose(RED_ALLIANCE, START_LEFT);

        // Construct a velocity constraint.
        TrajectoryVelocityConstraint velocityConstraint = new MecanumVelocityConstraint(MAXIMUM_VELOCITY, TRACK_WIDTH);

        // Construct an acceleration constraint.
        TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(MAXIMUM_ACCELERATION);

        // Construct a trajectory sequence builder.
        TrajectorySequenceBuilder trajectorySequenceBuilder = new TrajectorySequenceBuilder(startPose, null, velocityConstraint, accelerationConstraint, MAXIMUM_ANGULAR_VELOCITY, MAXIMUM_ANGULAR_ACCELERATION);

        // Drive to the spike mark.
        driveToSpikeMark(trajectorySequenceBuilder, RED_ALLIANCE, START_LEFT, LOCATION);

        // Wait for a bit.
        trajectorySequenceBuilder.waitSeconds(1);

        // Drive to the backdrop.
        driveToBackdrop(trajectorySequenceBuilder, RED_ALLIANCE, START_LEFT, LOCATION);

        // Wait for a bit.
        trajectorySequenceBuilder.waitSeconds(1);

        // Park.
        park(trajectorySequenceBuilder, PARK_LEFT);

        // Drive to a white pixel stack.
        //driveToStack(trajectorySequenceBuilder);

        // Build a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Construct a robot.
        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep).followTrajectorySequence(trajectorySequence);

        // Return the robot.
        return robot;

    }

    // Gets a start pose.
    private static Pose2d getStartPose(boolean redAlliance, boolean startLeft) {

        // Return a start pose.
        if(redAlliance) {
            if(startLeft) {
                return RED_LEFT_START;
            }
            else {
                return RED_RIGHT_START;
            }
        }
        else {
            if(startLeft) {
                return BLUE_LEFT_START;
            }
            else {
                return BLUE_RIGHT_START;
            }
        }

    }

    // Drives to a spike mark.
    public static void driveToSpikeMark(TrajectorySequenceBuilder trajectorySequenceBuilder, boolean redAlliance, boolean startLeft, TeamPropLocation location) {

        // Add the appropriate maneuvers.
        if (redAlliance) {
            if (startLeft) {
                if (location == LEFT) {
                    Pose2d targetPose = new Pose2d(-37.5, -26.5, Math.toRadians(180));
                    trajectorySequenceBuilder
                            .lineToLinearHeading(targetPose);
                }
                else if (location == MIDDLE) {
                    Pose2d targetPose = new Pose2d(-36, -14, Math.toRadians(-90));
                    trajectorySequenceBuilder
                            .lineToLinearHeading(targetPose);
                }
                else {
                    Pose2d targetPose = new Pose2d(-34.5, -29.5);
                    double targetHeading = Math.toRadians(0);
                    trajectorySequenceBuilder
                            .setReversed(true)
                            .splineToLinearHeading(targetPose, targetHeading);
                }
            }
            else {
                if (location == LEFT) {
                    trajectorySequenceBuilder
                            .splineToLinearHeading(new Pose2d(13,-30), Math.toRadians(180));
                }
                else if (location == MIDDLE) {
                    trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(12,-14, Math.toRadians(-90)));
                }
                else {
                    trajectorySequenceBuilder
                            .setReversed(true)
                            .lineToLinearHeading(new Pose2d(10,-30, Math.toRadians(0)));
                }
            }
        }
        else {
            if (startLeft) {
                if (location == LEFT) {
                    trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(13,30, Math.toRadians(180)));
                } else if (location == MIDDLE) {
                    trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(12,14, Math.toRadians(90)));
                } else {
                    trajectorySequenceBuilder
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(10,30), Math.toRadians(0));
                }
            } else {
                if (location == LEFT) {
                    trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(-34,30, Math.toRadians(180)));
                } else if (location == MIDDLE) {
                    trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(-36,14, Math.toRadians(90)));
                } else {
                    trajectorySequenceBuilder
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-37,30),Math.toRadians(0));
                }
            }
        }

    }

    // Drives to the backdrop.
    public static void driveToBackdrop(TrajectorySequenceBuilder trajectorySequenceBuilder, boolean redAlliance, boolean startLeft, TeamPropLocation location) {

        if (location == LEFT) {
            trajectorySequenceBuilder
                    .lineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(180)));
        } else if (location == MIDDLE) {

        } else {
            trajectorySequenceBuilder
                    .setReversed(true)
                    .splineTo(new Vector2d(-30, -13), Math.toRadians(0));
        }

        trajectorySequenceBuilder
                .setReversed(true)
                .splineTo(new Vector2d(0, -12), Math.toRadians(0))
                .splineTo(new Vector2d(28, -12), Math.toRadians(0))
                .splineTo(new Vector2d(44, -36), Math.toRadians(0));

    }

    // Drives to a white pixel stack.
    public static void driveToStack(TrajectorySequenceBuilder trajectorySequenceBuilder) {

        trajectorySequenceBuilder
                .setReversed(false)
                .splineTo(new Vector2d(28, -12), Math.toRadians(180))
                .splineTo(new Vector2d(0, -12), Math.toRadians(180))
                .splineTo(new Vector2d(-60, -10), Math.toRadians(180));

    }

    // Parks.
    public static void park(TrajectorySequenceBuilder trajectorySequenceBuilder, boolean parkLeft) {

        if (parkLeft) {
            trajectorySequenceBuilder
                    .setReversed(false)
                    .lineTo(new Vector2d(44, -12))
                    .lineTo(new Vector2d(60, -12));
        } else {
            trajectorySequenceBuilder
                    .setReversed(false)
                    .lineTo(new Vector2d(44, -60))
                    .lineTo(new Vector2d(60, -60));
        }

    }

}
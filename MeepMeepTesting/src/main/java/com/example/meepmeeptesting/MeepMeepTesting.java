package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Routes.driveToBackdrop;
import static com.example.meepmeeptesting.Routes.driveToSpikeMark;
import static com.example.meepmeeptesting.Routes.getStartPose;
import static com.example.meepmeeptesting.Routes.park;
import static com.example.meepmeeptesting.TeamPropLocation.LEFT;
import static com.example.meepmeeptesting.TeamPropLocation.MIDDLE;
import static com.example.meepmeeptesting.TeamPropLocation.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    private static final boolean PARK_LEFT = true;
    private static final TeamPropLocation LOCATION = MIDDLE;

    public static final double MAXIMUM_VELOCITY = 60;
    public static final double MAXIMUM_ACCELERATION = 60;
    public static final double MAXIMUM_ANGULAR_VELOCITY = Math.toRadians(180);
    public static final double MAXIMUM_ANGULAR_ACCELERATION = Math.toRadians(180);
    public static final double TRACK_WIDTH = 15;

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
        park(trajectorySequenceBuilder, RED_ALLIANCE, PARK_LEFT);

        // Drive to a white pixel stack.
        //driveToStack(trajectorySequenceBuilder);

        // Build a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Construct a robot.
        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep).followTrajectorySequence(trajectorySequence);

        // Return the robot.
        return robot;

    }

}
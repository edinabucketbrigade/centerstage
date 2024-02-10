package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.TeamPropLocation.LEFT;
import static com.example.meepmeeptesting.TeamPropLocation.MIDDLE;
import static com.example.meepmeeptesting.TeamPropLocation.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class Routes {

    // Gets a start pose.
    public static Pose2d getStartPose(boolean redAlliance, boolean startLeft) {

        // Return a start pose.
        if(redAlliance) {
            if(startLeft) {
                return new Pose2d(-36, -61, Math.toRadians(-90));
            }
            else {
                return new Pose2d(12, -61, Math.toRadians(-90));
            }
        }
        else {
            if(startLeft) {
                return new Pose2d(12, 61, Math.toRadians(90));
            }
            else {
                return new Pose2d(-36, 61, Math.toRadians(90));
            }
        }

    }

    // Drives to a spike mark.
    public static void driveToSpikeMark(TrajectorySequenceBuilder trajectorySequenceBuilder, boolean redAlliance, boolean startLeft, TeamPropLocation location) {

        // Add the appropriate maneuvers.
        if (redAlliance) {
            if (startLeft) {
                if (location == LEFT) {
                    trajectorySequenceBuilder
                            .lineTo(new Vector2d(-49, -19));
                }
                else if (location == MIDDLE) {
                    trajectorySequenceBuilder
                            .lineTo(new Vector2d(-36, -14));
                }
                else {
                    trajectorySequenceBuilder
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-34.5, -32.5), Math.toRadians(0));
                }
            }
            else {
                if (location == LEFT) {
                    trajectorySequenceBuilder
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(11,-27, Math.toRadians(180)), Math.toRadians(180));
                }
                else if (location == MIDDLE) {
                    trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(12,-14, Math.toRadians(-90)));
                }
                else {
                    trajectorySequenceBuilder
                            .lineTo(new Vector2d(22, -19));
                }
            }
        }
        else {
            if (startLeft) {
                if (location == LEFT) {
                    trajectorySequenceBuilder
                            //.lineTo(new Vector2d(25, 19));
                            .setReversed(true)
                            .splineTo(new Vector2d(32.5,30), Math.toRadians(0));
                } else if (location == MIDDLE) {
                    trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(12,14, Math.toRadians(90)));
                } else {
                    trajectorySequenceBuilder
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(11,33, Math.toRadians(180)), Math.toRadians(180));
                }
            } else {
                if (location == LEFT) {
                    trajectorySequenceBuilder
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-35,27),Math.toRadians(0));
                } else if (location == MIDDLE) {
                    trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(-36,14, Math.toRadians(90)));
                } else {
                    trajectorySequenceBuilder
                            .lineTo(new Vector2d(-46.5, 19));
                }
            }
        }

    }

    // Drives to the backdrop.
    public static void driveToBackdrop(TrajectorySequenceBuilder trajectorySequenceBuilder, boolean redAlliance, boolean startLeft, TeamPropLocation location, Vector2d targetVector) {

        // Construct a target pose.
        Pose2d targetPose = new Pose2d(targetVector, Math.toRadians(180));

        if (redAlliance) {
            if (startLeft) {
                trajectorySequenceBuilder.lineTo(new Vector2d(-36, -9));
                if(location == LEFT || location == MIDDLE) {
                    trajectorySequenceBuilder.turn(Math.toRadians(-90));
                }
                else {
                    trajectorySequenceBuilder.turn(Math.toRadians(180));
                }
                trajectorySequenceBuilder
                        .lineTo(new Vector2d(30, -9))
                        .splineTo(targetVector, Math.toRadians(0));
            } else {
                if(location == MIDDLE) {
                    trajectorySequenceBuilder.back(10);
                }
                else if(location == RIGHT) {
                    trajectorySequenceBuilder.back(14);
                }
                trajectorySequenceBuilder.lineToLinearHeading(targetPose);
            }
        } else {
            if (startLeft) {
                if(location == MIDDLE) {
                    trajectorySequenceBuilder.back(10);
                }
                trajectorySequenceBuilder.lineToLinearHeading(targetPose);
            } else {
                trajectorySequenceBuilder.lineTo(new Vector2d(-36, 9));
                if(location == MIDDLE || location == RIGHT) {
                    trajectorySequenceBuilder.turn(Math.toRadians(90));
                }
                else {
                    trajectorySequenceBuilder.turn(Math.toRadians(180));
                }
                trajectorySequenceBuilder
                        .lineTo(new Vector2d(30, 9))
                        .splineTo(targetVector, Math.toRadians(0));
            }
        }

    }

    // Drives to a white pixel stack.
    public static void driveToStack(TrajectorySequenceBuilder trajectorySequenceBuilder, boolean redAlliance) {

        if(redAlliance) {

        }
        else {
            trajectorySequenceBuilder
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(20,9, Math.toRadians(180)), Math.toRadians(180))
                    .lineTo(new Vector2d(-57,9));
        }

    }

    // Parks.
    public static void park(TrajectorySequenceBuilder trajectorySequenceBuilder, boolean redAlliance, boolean parkLeft) {

        if(redAlliance) {
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
        else {
            if (parkLeft) {
                trajectorySequenceBuilder
                        .setReversed(false)
                        .lineTo(new Vector2d(44, 60))
                        .lineTo(new Vector2d(60, 60));
            } else {
                trajectorySequenceBuilder
                        .setReversed(false)
                        .lineTo(new Vector2d(44, 12))
                        .lineTo(new Vector2d(60, 12));
            }
        }

    }

    public static void returnToBackdrop(TrajectorySequenceBuilder trajectorySequenceBuilder, boolean redAlliance, Vector2d targetVector) {
        if(redAlliance) {

        }
        else {
            trajectorySequenceBuilder
                    .lineTo(new Vector2d(20,9))
                    .splineToLinearHeading(new Pose2d(targetVector.getX(),targetVector.getY(), Math.toRadians(180)), Math.toRadians(90));
        }
    }

}
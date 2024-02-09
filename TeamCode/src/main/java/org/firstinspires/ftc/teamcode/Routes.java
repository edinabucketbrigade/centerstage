package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamPropLocation.LEFT;
import static org.firstinspires.ftc.teamcode.TeamPropLocation.MIDDLE;
import static org.firstinspires.ftc.teamcode.TeamPropLocation.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

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
                    Vector2d targetVector = new Vector2d(-49, -19);
                    trajectorySequenceBuilder
                            .lineTo(targetVector);
                }
                else if (location == MIDDLE) {
                    Vector2d targetVector = new Vector2d(-36, -14);
                    trajectorySequenceBuilder
                            .lineTo(targetVector);
                }
                else {
                    Pose2d targetPose = new Pose2d(-34.5, -32.5);
                    double targetHeading = Math.toRadians(0);
                    trajectorySequenceBuilder
                            .setReversed(true)
                            .splineToLinearHeading(targetPose, targetHeading);
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
                            .setReversed(true)
                            .lineToLinearHeading(new Pose2d(13,-33, Math.toRadians(0)));
                }
            }
        }
        else {
            if (startLeft) {
                if (location == LEFT) {
                    trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(13,27, Math.toRadians(0)));
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
                            .lineToLinearHeading(new Pose2d(-37,33, Math.toRadians(180)));
                }
            }
        }

    }

    // Drives to the backdrop.
    public static void driveToBackdrop(TrajectorySequenceBuilder trajectorySequenceBuilder, boolean redAlliance, boolean startLeft, TeamPropLocation location) {

        if (redAlliance) {
            if (startLeft) {
                trajectorySequenceBuilder.lineTo(new Vector2d(-36, -9));
                if(location == LEFT) {
                    trajectorySequenceBuilder.turn(Math.toRadians(-90));
                }
                else if(location == MIDDLE) {
                    trajectorySequenceBuilder.turn(Math.toRadians(-90));
                }
                else {
                    trajectorySequenceBuilder.turn(Math.toRadians(180));
                }
                trajectorySequenceBuilder
                        .lineTo(new Vector2d(30, -9))
                        .splineTo(new Vector2d(44, -36), Math.toRadians(0));
            } else {
                trajectorySequenceBuilder
                        .lineToLinearHeading(new Pose2d(44, -36, Math.toRadians(180)));
            }
        } else {
            if (startLeft) {
                trajectorySequenceBuilder
                        .lineToLinearHeading(new Pose2d(44, 36, Math.toRadians(180)));
            } else {
                trajectorySequenceBuilder.lineTo(new Vector2d(-36, 9));
                if(location == MIDDLE) {
                    trajectorySequenceBuilder.turn(Math.toRadians(90));
                }
                else if(location == LEFT) {
                    trajectorySequenceBuilder.turn(Math.toRadians(180));
                }
                trajectorySequenceBuilder
                        .lineTo(new Vector2d(30, 9))
                        .splineTo(new Vector2d(44, 36), Math.toRadians(0));
            }
        }

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

}
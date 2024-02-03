package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

enum Route {
    RED_LEFT_DIRECT,
    RED_LEFT_INDIRECT,
    RED_RIGHT_DIRECT,
    RED_RIGHT_INDIRECT,
    BLUE_LEFT_DIRECT,
    BLUE_LEFT_INDIRECT,
    BLUE_RIGHT_DIRECT,
    BLUE_RIGHT_INDIRECT,
    SPEED_TEST,
    CYCLE_TEST,
    PARK_LEFT,
    PARK_RIGHT

}

public class MeepMeepTesting {

    public static final Route ROUTE = Route.RED_LEFT_DIRECT;
    public static final double DELAY = 0.5;
    public static final double MAXVEL = 60;
    public static final double MAXACCEL = 60;
    public static final double MAXANGVEL = Math.toRadians(180);
    public static final double MAXANGACCEL = Math.toRadians(180);
    public static final double TRACKWIDTH = 15;
    

    public static void main(String[] args) throws Exception {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot;

        switch (ROUTE) {
            case RED_LEFT_DIRECT:
                myBot = redLeftDirect(meepMeep);
                break;
            case RED_LEFT_INDIRECT:
                myBot = redLeftIndirect(meepMeep);
                break;
            case RED_RIGHT_DIRECT:
                myBot = redRightDirect(meepMeep);
                break;
            case RED_RIGHT_INDIRECT:
                myBot = redRightIndirect(meepMeep);
                break;
            case BLUE_LEFT_DIRECT:
                myBot = blueLeftDirect(meepMeep);
                break;
            case BLUE_LEFT_INDIRECT:
                myBot = blueLeftIndirect(meepMeep);
                break;
            case BLUE_RIGHT_DIRECT:
                myBot = blueRightDirect(meepMeep);
                break;
            case BLUE_RIGHT_INDIRECT:
                myBot = blueRightIndirect(meepMeep);
                break;
            case CYCLE_TEST:
                myBot = cycleTest(meepMeep);
                break;
            case SPEED_TEST:
                myBot = speedTest(meepMeep);
                break;
            case PARK_LEFT:
                myBot = parkLeft(meepMeep);
                break;
            case PARK_RIGHT:
                myBot = parkRight(meepMeep);
                break;
            default:
                throw new Exception("unrecognized route");
        }

        /*
        Image img = null;
        try {
            img = ImageIO.read(new File("field-2023-official.png"));
        } catch (IOException e) {
        }
        */
        //meepMeep.setBackground(img)
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


    }
    private static RoadRunnerBotEntity redLeftDirect(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                // Red right start X
                // 12

                // Red left start X
                // -36
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, -61, Math.toRadians(-90)))
                    // Red right
                            // Left
                                //.lineToLinearHeading(new Pose2d(14,-30, Math.toRadians(0)))
                            // Middle
                                //.back(26)
                            // Right
//                                .setReversed(true)
//                                .splineTo(new Vector2d(23,-30), Math.toRadians(0))
                            // Other thing
                                //.lineToLinearHeading(new Pose2d(44,-36, Math.toRadians(180)))

                        // Red left
                                // Left
//                                .lineToLinearHeading(new Pose2d(-34,-34,Math.toRadians(0)))
//                                .strafeLeft(20)
//                                .turn(Math.toRadians(-90))
                                // Middle
                                //.back(42)
                                // Right
//                                .back(18)
//                                .setReversed(true)
//                                .splineTo(new Vector2d(-9.5, -35),Math.toRadians(0))
//                                .turn(Math.toRadians(90))
                                // Other thing
//                                .setReversed(true)
//                                .splineTo(new Vector2d(0,-8),Math.toRadians(0))
//                                .splineTo(new Vector2d(28,-8), Math.toRadians(0))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(44,-36), Math.toRadians(0))

                        // Rest of the route (shortened version for meep meep)
//                                .setReversed(false)
//                                .splineTo(new Vector2d(28,-8), Math.toRadians(180))
//                                .splineTo(new Vector2d(0, -8),Math.toRadians(180))
//                                .splineTo(new Vector2d(-52, -12), Math.toRadians(180))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(0, -8), Math.toRadians(0))
//                                .splineTo(new Vector2d(28,-8), Math.toRadians(0))
//                                .splineTo(new Vector2d(44, -36), Math.toRadians(0))


                                //.lineToLinearHeading(new Pose2d(-36,-10, Math.toRadians(-90)))

                                .setReversed(true)

                                .splineToLinearHeading(new Pose2d(-37,-30),Math.toRadians(0))

                                //.lineToLinearHeading(new Pose2d(-34,-30,Math.toRadians(180)))

                                .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity redLeftIndirect(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, -36, Math.toRadians(180)))
                        .forward(26)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(35, -56, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(6, -24, Math.toRadians(90)))
                        .forward(74)
                        .strafeRight(29)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(35, 0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(35, -56, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(6, -24, Math.toRadians(90)))
                        .forward(74)
                        .strafeRight(29)
                        .waitSeconds(DELAY)
                        .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity redRightDirect(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, 12, Math.toRadians(180)))
                        .forward(26)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(35, 50, Math.toRadians(90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(35, 0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(35, -56, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(35, -24, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(35, 50, Math.toRadians(90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(35, 0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(35, -56, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(35, -24, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(35, 50, Math.toRadians(90)))
                        .waitSeconds(DELAY)
                        .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity redRightIndirect(MeepMeep meepMeep){
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, 12, Math.toRadians(180)))
                        .forward(26)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(35, 50, Math.toRadians(90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(10, 5, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(11, -58, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .back(70)
                        .lineToLinearHeading(new Pose2d(35, 50, Math.toRadians(90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(10, 5, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(11, -58, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .back(70)
                        .lineToLinearHeading(new Pose2d(35, 50, Math.toRadians(90)))
                        .waitSeconds(DELAY)
                        .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity blueLeftDirect(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, 61, Math.toRadians(90)))
                        .back(26)
                        .lineToLinearHeading(new Pose2d(44,36, Math.toRadians(180)))
                        .splineTo(new Vector2d(18,8),Math.toRadians(180))
                        .splineTo(new Vector2d(0,8),Math.toRadians(180))
                        .splineTo(new Vector2d(-58,10), Math.toRadians(180))
                        .setReversed(true)
                        .splineTo(new Vector2d(0,8),Math.toRadians(0))
                        .splineTo(new Vector2d(18,8),Math.toRadians(0))
                        .splineTo(new Vector2d(44,36),Math.toRadians(0))
                        .setReversed(false)
                        .splineTo(new Vector2d(18,8),Math.toRadians(180))
                        .splineTo(new Vector2d(0,8),Math.toRadians(180))
                        .splineTo(new Vector2d(-58,10), Math.toRadians(180))
                        .setReversed(true)
                        .splineTo(new Vector2d(0,8),Math.toRadians(0))
                        .splineTo(new Vector2d(18,8),Math.toRadians(0))
                        .splineTo(new Vector2d(44,36),Math.toRadians(0))
                        .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity blueLeftIndirect(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-61, 12, Math.toRadians(0)))
                        .forward(26)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(-35, 50, Math.toRadians(90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(-10, 5, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-11, -58, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .back(70)
                        .lineToLinearHeading(new Pose2d(-35, 50, Math.toRadians(90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(-10, 5, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-11, -58, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .back(70)
                        .lineToLinearHeading(new Pose2d(-35, 50, Math.toRadians(90)))
                        .waitSeconds(DELAY)
                        .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity blueRightDirect(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-61, -36, Math.toRadians(0)))
                        .forward(26)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(-35, -56, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(-35, -24, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-35, 50, Math.toRadians(90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(-35, 0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-35, -56, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(-35, -24, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-35, 50, Math.toRadians(90)))
                        .waitSeconds(DELAY)
                        .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity blueRightIndirect(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-61, -36, Math.toRadians(0)))
                        .forward(26)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(-35, -56, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(-6, -24, Math.toRadians(90)))
                        .forward(74)
                        .strafeRight(-29)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(-35, 0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-35, -56, Math.toRadians(-90)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(-6, -24, Math.toRadians(90)))
                        .forward(74)
                        .strafeRight(-29)
                        .waitSeconds(DELAY)
                        .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity cycleTest(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(35, -56, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(35, -24, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(35, 50, Math.toRadians(90)))
                        .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity speedTest(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-61, -36, Math.toRadians(0)))
                        .forward(123)
                        .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity parkLeft(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(35, 50, Math.toRadians(90)))
                        .strafeRight(-24)
                        .forward(10)
                        .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity parkRight(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGVEL, MAXANGACCEL, TRACKWIDTH)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-35, 50, Math.toRadians(90)))
                        .strafeRight(24)
                        .forward(10)
                        .build()
                );
        return myBot;
    }

}

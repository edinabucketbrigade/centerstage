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

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                //Red left
               /* .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, -36, Math.toRadians(180)))
                        .forward(26)
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(35, -56, Math.toRadians(-90)))
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(6, -24, Math.toRadians(90)))
                        .forward(74)
                        .strafeRight(29)
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(35, 0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(35, -56, Math.toRadians(-90)))
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(6, -24, Math.toRadians(90)))
                        .forward(74)
                        .strafeRight(29)
                        .waitSeconds(0.5)
                        .strafeRight(24)
                        .forward(10)
                        .build()*/

                //Blue right
                /*.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-61, -36, Math.toRadians(0)))
                        .forward(26)
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(-35, -56, Math.toRadians(-90)))
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(-6, -24, Math.toRadians(90)))
                        .forward(74)
                        .strafeRight(-29)
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(-35, 0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-35, -56, Math.toRadians(-90)))
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(-6, -24, Math.toRadians(90)))
                        .forward(74)
                        .strafeRight(-29)
                        .waitSeconds(0.5)
                        .strafeRight(-24)
                        .forward(10)
                        .build()*/

                //Red right
                /*.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, 12, Math.toRadians(180)))
                                .forward(26)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(35, 50, Math.toRadians(90)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(10, 5, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(11, -58, Math.toRadians(-90)))
                                .waitSeconds(0.5)
                                .back(70)
                                .lineToLinearHeading(new Pose2d(35, 50, Math.toRadians(90)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(10, 5, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(11, -58, Math.toRadians(-90)))
                                .waitSeconds(0.5)
                                .back(70)
                                .lineToLinearHeading(new Pose2d(35, 50, Math.toRadians(90)))
                                .waitSeconds(0.5)
                                .strafeRight(24)
                                .forward(10)
                                .build()*/

                //Speed test
               .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-61, -36, Math.toRadians(0)))
                        .forward(123)
                        .build()

                //Cycle test
                /*.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(35, -56, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(35, -24, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(35, 50, Math.toRadians(90)))
                        .build()*/
                );
        Image img = null;
        try { img = ImageIO.read(new File("field-2023-official.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
        //meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
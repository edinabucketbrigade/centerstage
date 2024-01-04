package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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

@Autonomous(preselectTeleOp = "TeleOpA")
//@Disabled
public class AutoF extends LinearOpMode {
    public static final Route ROUTE = Route.RED_LEFT_DIRECT;
    public static final double DELAY = 0.5;
    @Override
    public void runOpMode() {
/*
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d(61, -36, Math.toRadians(180)))
                .lineTo(new Vector2d(35, -36))
                //.lineToLinearHeading(new Pose2d(35, 36, Math.toRadians(180)))
                .splineTo(new Vector2d(35, 36), Math.toRadians(90))
                .build();

        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d(61, -36, Math.toRadians(180)))
                .back(26)
                .turn(Math.toRadians(90))
                .forward(72)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajectorySequence);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
        */

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-24, -24, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        //while (!isStopRequested()) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .back(26)
                    .turn(Math.toRadians(-90))
                    .back(72)
                    /*.forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))*/
                    .build();
            drive.followTrajectorySequence(trajSeq);
        //}

    }

}
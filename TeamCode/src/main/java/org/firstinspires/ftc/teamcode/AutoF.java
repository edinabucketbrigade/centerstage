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
    public static final double RED_MIDDLE_X = 0;
    public static final double RED_MIDDLE_Y = -8;
    public static final double RED_BACKDROP_X = 40;
    public static final double RED_BACKDROP_Y = -36;
    public static final double RED_PIXELS_X = -58;
    public static final double RED_PIXELS_Y = -10;
    public static final double RED_LEFT_START_X = -36;
    public static final double RED_LEFT_START_Y = -61;
    public static final double RED_RIGHT_START_X = 12;
    public static final double RED_RIGHT_START_Y = -61;



    public static final Route ROUTE = Route.RED_LEFT_DIRECT;
    public static final double DELAY = 0.5;
    @Override
    public void runOpMode() {
/*
        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) ;
        */

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        //while (!isStopRequested()) {

        TrajectorySequence sequence = getRedRightTrajectorySequence(drive);
        drive.followTrajectorySequence(sequence);
        //}

    }

    private TrajectorySequence getRedLeftTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_LEFT_START_X, RED_LEFT_START_Y, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(32)
                .setReversed(true)
                .splineTo(new Vector2d(RED_MIDDLE_X,RED_MIDDLE_Y),Math.toRadians(0))
                .splineTo(new Vector2d(RED_BACKDROP_X,RED_BACKDROP_Y), Math.toRadians(0))
                // Heat seek
                .setReversed(false)
                .splineTo(new Vector2d(RED_MIDDLE_X,RED_MIDDLE_Y),Math.toRadians(180))
                .splineTo(new Vector2d(RED_PIXELS_X,RED_PIXELS_Y), Math.toRadians(180))
                .setReversed(true)
                .splineTo(new Vector2d(RED_MIDDLE_X,RED_MIDDLE_Y),Math.toRadians(0))
                .splineTo(new Vector2d(RED_BACKDROP_X,RED_BACKDROP_Y),Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(RED_MIDDLE_X,RED_MIDDLE_Y),Math.toRadians(180))
                .splineTo(new Vector2d(RED_PIXELS_X,RED_PIXELS_Y), Math.toRadians(180))
                .setReversed(true)
                .splineTo(new Vector2d(RED_MIDDLE_X,RED_MIDDLE_Y),Math.toRadians(0))
                .splineTo(new Vector2d(RED_BACKDROP_X,RED_BACKDROP_Y),Math.toRadians(0))
                .build();
        return sequence;
    }

    private TrajectorySequence getRedRightTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_RIGHT_START_X, RED_RIGHT_START_Y, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .lineToLinearHeading(new Pose2d(RED_BACKDROP_X, RED_BACKDROP_X, Math.toRadians(-90)))
                .setReversed(false)
                .splineTo(new Vector2d(RED_MIDDLE_X,RED_MIDDLE_Y),Math.toRadians(-90))
                .splineTo(new Vector2d(RED_PIXELS_X,RED_PIXELS_Y), Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(RED_MIDDLE_X,RED_MIDDLE_Y),Math.toRadians(90))
                .splineTo(new Vector2d(RED_BACKDROP_X,RED_BACKDROP_Y),Math.toRadians(90))
                .setReversed(false)
                .splineTo(new Vector2d(RED_MIDDLE_X,RED_MIDDLE_Y),Math.toRadians(-90))
                .splineTo(new Vector2d(RED_PIXELS_X,RED_PIXELS_Y), Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(RED_MIDDLE_X,RED_MIDDLE_Y),Math.toRadians(90))
                .splineTo(new Vector2d(RED_BACKDROP_X,RED_BACKDROP_Y),Math.toRadians(90))
                .build();
        return sequence;
    }

}
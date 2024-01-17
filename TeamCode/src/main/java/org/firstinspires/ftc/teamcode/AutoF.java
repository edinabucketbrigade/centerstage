package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    public static final Vector2d RED_MIDDLE = new Vector2d(0,-8);
    public  static final Vector2d RED_DETOUR_BACKDROP = new Vector2d(28,-8);
    public static final Vector2d RED_BACKDROP = new Vector2d(44,-36);
    public static final Vector2d RED_PIXELS = new Vector2d(-58,-10);
    public static final Vector2d RED_LEFT_START = new Vector2d(-36,-61);
    public static final Vector2d RED_RIGHT_START = new Vector2d(12,-61);

    public static final Vector2d BLUE_MIDDLE = new Vector2d(0,8);
    public  static final Vector2d BLUE_DETOUR_BACKDROP = new Vector2d(28,8);
    public static final Vector2d BLUE_BACKDROP = new Vector2d(44,36);
    public static final Vector2d BLUE_PIXELS = new Vector2d(-58,10);
    public static final Vector2d BLUE_LEFT_START = new Vector2d(12,61);
    public static final Vector2d BLUE_RIGHT_START = new Vector2d(-36,61);

    private Boolean redAlliance = null;
    private Boolean startLeft = null;
    private Boolean parkLeft = null;



    public static final Route ROUTE = Route.RED_LEFT_DIRECT;
    public static final double DELAY = 0.5;
    @Override
    public void runOpMode() {
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        while (true) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (redAlliance == null) {
                telemetry.addData("Alliance", "X = blue, B = red");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    redAlliance = false;
                }
                if (currentGamepad.b && !previousGamepad.b) {
                    redAlliance = true;
                }
            } else if (startLeft == null) {
                telemetry.addData("Start", "X = left, B = right");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    startLeft = true;
                }
                if (currentGamepad.b && !previousGamepad.b) {
                    startLeft = false;
                }
            } else if (parkLeft == null) {
                telemetry.addData("Park", "X = left, B = right");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    parkLeft = true;
                }
                if (currentGamepad.b && !previousGamepad.b) {
                    parkLeft = false;
                }
            } else {
                break;
            }
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence redLeftSequence = getRedLeftMiddleTrajectorySequence(drive);
        TrajectorySequence redRightSequence = getRedRightMiddleTrajectorySequence(drive);
        TrajectorySequence blueLeftSequence = getRedLeftMiddleTrajectorySequence(drive);
        TrajectorySequence blueRightSequence = getRedRightMiddleTrajectorySequence(drive);


        if (redAlliance) {
            if (startLeft) {
                drive.followTrajectorySequence(redLeftSequence);
            }
            else {
                drive.followTrajectorySequence(redRightSequence);
            }
        }
        else {
            if (startLeft) {
                drive.followTrajectorySequence(blueLeftSequence);
            }
            else {
                drive.followTrajectorySequence(blueRightSequence);
            }
        }
    }

    private TrajectorySequence getRedLeftMiddleTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_LEFT_START, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(32)
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_MIDDLE,Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP,Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_MIDDLE,Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP,Math.toRadians(0))
                .build();
        return sequence;
    }

    private TrajectorySequence getRedRightMiddleTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .lineToLinearHeading(new Pose2d(RED_BACKDROP, Math.toRadians(180)))
                .setReversed(false)
                .splineTo(RED_MIDDLE,Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP,Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_MIDDLE,Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP,Math.toRadians(0))
                .build();
        return sequence;
    }

    private TrajectorySequence getRedRightLeftTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(14,-30, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(RED_BACKDROP, Math.toRadians(180)))
                .setReversed(false)
                .splineTo(RED_MIDDLE,Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP,Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_MIDDLE,Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP,Math.toRadians(0))
                .build();
        return sequence;
    }


    private TrajectorySequence getBlueRightTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_RIGHT_START, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .lineToLinearHeading(new Pose2d(BLUE_BACKDROP, Math.toRadians(180)))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                .build();
        return sequence;
    }

    private TrajectorySequence getBlueLeftTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_LEFT_START, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(32)
                .setReversed(true)
                .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                .build();
        return sequence;
    }

}
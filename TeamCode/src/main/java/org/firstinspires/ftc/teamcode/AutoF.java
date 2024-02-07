package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoF.State.APPROACH_BACKDROP;
import static org.firstinspires.ftc.teamcode.AutoF.State.DRIVE_TO_PLACE_POSITION;
import static org.firstinspires.ftc.teamcode.AutoF.State.IDLE;
import static org.firstinspires.ftc.teamcode.AutoF.State.DRIVE_TO_SPIKE_MARK;
import static org.firstinspires.ftc.teamcode.AutoF.State.RAISE_ARM_LIFT_AND_WRIST;
import static org.firstinspires.ftc.teamcode.AutoF.State.RELEASE_PURPLE_PIXEL;
import static org.firstinspires.ftc.teamcode.AutoF.State.RELEASE_YELLOW_PIXEL;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(preselectTeleOp = "TeleOpT")
public class AutoF extends LinearOpMode {

    enum Route {
        RED_LEFT_DIRECT,
        RED_LEFT_INDIRECT,
        RED_RIGHT_DIRECT,
        RED_RIGHT_INDIRECT,
        BLUE_LEFT_DIRECT,
        BLUE_LEFT_INDIRECT,
        BLUE_RIGHT_DIRECT,
        BLUE_RIGHT_INDIRECT,
    }

    public static final Vector2d RED_MIDDLE = new Vector2d(0, -12);
    public static final Vector2d RED_DETOUR_BACKDROP = new Vector2d(28, -12);
    public static final Vector2d RED_BACKDROP = new Vector2d(44, -36);
    public static final Vector2d RED_PIXELS = new Vector2d(-58, -10);
    public static final Pose2d RED_LEFT_START = new Pose2d(-36, -61, Math.toRadians(-90));
    public static final Pose2d RED_RIGHT_START = new Pose2d(12, -61, Math.toRadians(-90));
    public static final Vector2d RED_RIGHT_LEFT_POSITION = new Vector2d(14, -30);
    public static final Vector2d RED_RIGHT_RIGHT_POSITION = new Vector2d(23, -30);
    public static final Vector2d RED_LEFT_RIGHT_RIGGING_POSITION = new Vector2d(-9.5, -35);
    public static final Vector2d RED_LEFT_LEFT_SPIKE_MARK_POSITION = new Vector2d(-34, -34);
    public static final Vector2d BLUE_MIDDLE = new Vector2d(0, 12);
    public static final Vector2d BLUE_DETOUR_BACKDROP = new Vector2d(28, 12);
    public static final Vector2d BLUE_BACKDROP = new Vector2d(44, 36);
    public static final Vector2d BLUE_PIXELS = new Vector2d(-58, 10);
    public static final Pose2d BLUE_LEFT_START = new Pose2d(12, 61, Math.toRadians(90));
    public static final Pose2d BLUE_RIGHT_START = new Pose2d(-36, 61, Math.toRadians(90));
    public static final Vector2d BLUE_RIGHT_LEFT_POSITION = new Vector2d(23, 30);
    public static final Vector2d BLUE_RIGHT_RIGHT_POSITION = new Vector2d(14, 30);
    public static final Vector2d BLUE_LEFT_LEFT_RIGGING_POSITION = new Vector2d(-9.5, 35);
    public static final Vector2d BLUE_LEFT_RIGHT_SPIKE_MARK_POSITION = new Vector2d(-34, 34);
    public static final Route ROUTE = Route.RED_LEFT_DIRECT;
    public static final double DELAY = 0.5;
    private static final String TAG = "Bucket Brigade";
    public static final int FIRST_ROW = 1;
    public static double PURPLE_PIXEL_DELAY = 1;

    public static double RED_START_LEFT_SPIKE_LEFT_X = -37.5;
    public static double RED_START_LEFT_SPIKE_LEFT_Y = -26.5;
    public static double RED_START_LEFT_SPIKE_LEFT_HEADING = 180;
    public static double RED_START_LEFT_SPIKE_MIDDLE_X = -36;
    public static double RED_START_LEFT_SPIKE_MIDDLE_Y = -14;
    public static double RED_START_LEFT_SPIKE_MIDDLE_HEADING = -90;
    public static double RED_START_LEFT_SPIKE_RIGHT_X = -34.5;
    public static double RED_START_LEFT_SPIKE_RIGHT_Y = -29.5;
    public static double RED_START_LEFT_SPIKE_RIGHT_HEADING = 0;

    public static final double RED_MIDDLE_X = 0;
    public static final double RED_MIDDLE_Y = -12;
    public static final double RED_TOWARDS_BACKDROP_HEADING = 0;
    public static final double RED_DETOUR_BACKDROP_X = 28;
    public static final double RED_DETOUR_BACKDROP_Y = -12;
    public static final double RED_BACKDROP_X = 44;
    public static final double RED_BACKDROP_Y = -36;
    public static final double RED_PIXELS_X = -58;
    public static final double RED_PIXELS_Y = -10;
    public static final double RED_TOWARDS_PIXELS_HEADING = 180;

    public static Boolean redAlliance;
    public static Pose2d currentPose;
    public static boolean lastRanAutonomous;
    private Boolean startLeft;
    private Boolean parkLeft;
    private OpenCvWebcam camera;
    private boolean startedStreaming;
    private Gamepad previousGamepad = new Gamepad();
    private Gamepad currentGamepad = new Gamepad();
    private CenterStageCVDetection.Location location;
    private CenterStageCVDetection teamPropDetector;
    private RobotHardwareC robotHardware;
    enum State { IDLE, DRIVE_TO_SPIKE_MARK, RELEASE_PURPLE_PIXEL, APPROACH_BACKDROP, RAISE_ARM_LIFT_AND_WRIST, DRIVE_TO_PLACE_POSITION, RELEASE_YELLOW_PIXEL, RETRACT }
    private State state = IDLE;
    private ElapsedTime timer = new ElapsedTime();
    private Pose2d lastEnd;

    @Override
    public void runOpMode() throws InterruptedException {

        // Clear the red alliance value.
        redAlliance = null;

        // Remember that we last ran autonomous.
        lastRanAutonomous = true;

        // Get the robot hardware.
        robotHardware = new RobotHardwareC(this);

        // Wait for menu selection.
        waitForMenuSelection();

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        //robotHardware.waitForArmDown();

        // Wait for the camera to open.
        waitForCameraOpen();

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Close the claw.
        robotHardware.closeClaw();

        // Wait a bit.
        sleep(2000);

        // Move the wrist to the backdrop position.
        robotHardware.setWristBackdrop();

        // Notify the user that we are waiting for start.
        log("Waiting for start...");

        // Wait for start.
        waitForStart();

        // Wait for team prop detection.
        waitForTeamPropDetection();

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Start looking for AprilTags.
        robotHardware.startLookingForAprilTags();

        // Initialize the drive interface.
        robotHardware.initializeDrive();

        // Start the state machine.
        setState(DRIVE_TO_SPIKE_MARK);

        // While the op mode is active...
        while (!isStopRequested()) {

            // Update this.
            update();

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

        }

    }

    // Updates the state machine.
    public void update() throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Update the robot's current pose.
        currentPose = robotHardware.getPose();

        // Update the state machine.
        switch (state) {

            case DRIVE_TO_SPIKE_MARK:

                // Get a spike mark trajectory sequence.
                TrajectorySequence spikeMarkTrajectorySequence = getSpikeMarkTrajectorySequence();
                lastEnd = spikeMarkTrajectorySequence.end();

                // Start driving to the spike mark.
                drive.followTrajectorySequenceAsync(spikeMarkTrajectorySequence);

                // Move the wrist to the ground position.
                robotHardware.setWristGround();

                // Advance to the next step.
                setState(RELEASE_PURPLE_PIXEL);

                break;

            case RELEASE_PURPLE_PIXEL:

                // If the robot is driving...
                if (drive.isBusy()) {

                    // Exit the method.
                    return;

                }

                // Open the left claw.
                robotHardware.openLeftClawFully();

                // Advance to the next step.
                setState(APPROACH_BACKDROP);

                break;

            case APPROACH_BACKDROP:

                if (timer.milliseconds() > 500) {
                    return;
                }

                TrajectorySequence backdropTrajectorySequence = getBackdropTrajectorySequence();
                lastEnd = backdropTrajectorySequence.end();

                // Start driving to the backdrop.
                drive.followTrajectorySequenceAsync(backdropTrajectorySequence);

                // Advance to the next step.
                setState(RAISE_ARM_LIFT_AND_WRIST);

                break;

            case RAISE_ARM_LIFT_AND_WRIST:

                if (drive.isBusy()) {
                    return;
                }

                // Get a lift position.
                int liftPosition = HeatSeekC.getTargetLiftPosition(1);

                // Raise the lift.
                robotHardware.raiseLift(liftPosition);

                // Move the wrist to the backdrop position.
                robotHardware.setWristBackdrop();

                // Raise the arm.
                robotHardware.raiseArm();

                setState(DRIVE_TO_PLACE_POSITION);

                break;

            case DRIVE_TO_PLACE_POSITION:

                // If we are waiting...
                if (!robotHardware.isArmUp() && !robotHardware.isLiftUp()) {

                    // Exit the method.
                    return;

                }

                // Construct a velocity constraint.
                TrajectoryVelocityConstraint velocityConstraint = new MecanumVelocityConstraint(HeatSeekC.PLACE_SPEED, DriveConstants.TRACK_WIDTH);

                // Construct an acceleration constraint.
                TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(HeatSeekC.PLACE_SPEED);

                // Get the robot's current pose.
                Pose2d currentPose = drive.getPoseEstimate();

                int leftColumn = 1;
                int row = 1;

                // Get a target y coordinate.
                double targetY = HeatSeekC.getTargetY(leftColumn, row, redAlliance);

                // Construct a target position.
                Vector2d targetPosition = new Vector2d(HeatSeekC.PLACE_TARGET_X, targetY);

                // Construct a target pose.
                Pose2d targetPose = new Pose2d(targetPosition, Math.toRadians(180));

                // Construct a trajectory sequence.
                TrajectorySequence sequence = drive.trajectorySequenceBuilder(currentPose)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineToLinearHeading(targetPose)
                        .build();

                // Execute the trajectory sequence.
                drive.followTrajectorySequenceAsync(sequence);

                setState(RELEASE_YELLOW_PIXEL);

                break;

            case RELEASE_YELLOW_PIXEL:

                // Open the left claw.
                robotHardware.openClawPartially();

                setState(IDLE);

                break;

            case RETRACT:

                // Start retracting.
                robotHardware.startRetracting();

            case IDLE:

                break;

            default:

                throw new InterruptedException("Unrecognized state");

        }

    }

    // Converts a pose to a string.
    public static String toString(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double radians = pose.getHeading();
        double degrees = Math.toDegrees(radians);
        String output = String.format("x %.2f, y %.2f, heading %.2f", x, y, degrees);
        return output;
    }
/*
    private TrajectorySequence getRedLeftLeftTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(RED_LEFT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(RED_LEFT_LEFT_SPIKE_MARK_POSITION, Math.toRadians(0)))
                .strafeLeft(20)
                .turn(Math.toRadians(-90))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_MIDDLE, Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE, Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_MIDDLE, Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE, Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Middle
    private TrajectorySequence getRedLeftMiddleTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(RED_LEFT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(42)
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_MIDDLE,Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE,Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP,Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_MIDDLE,Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE,Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP,Math.toRadians(0))
                .build();
        return sequence;
    }

    // Right
    private TrajectorySequence getRedLeftRightTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(RED_LEFT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(18)
                .setReversed(true)
                .splineTo(RED_LEFT_RIGHT_RIGGING_POSITION, Math.toRadians(0))
                .turn(Math.toRadians(90))
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_MIDDLE, Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE, Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_MIDDLE, Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE, Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Right

    // Left
    private TrajectorySequence getRedRightLeftTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(RED_RIGHT_LEFT_POSITION, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(RED_BACKDROP, Math.toRadians(180)))
//                .setReversed(false)
//                .splineTo(RED_MIDDLE, Math.toRadians(180))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE, Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(RED_MIDDLE, Math.toRadians(180))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE, Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Middle
    private TrajectorySequence getRedRightMiddleTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .lineToLinearHeading(new Pose2d(RED_BACKDROP, Math.toRadians(180)))
//                .setReversed(false)
//                .splineTo(RED_MIDDLE, Math.toRadians(180))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE, Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(RED_MIDDLE, Math.toRadians(180))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE, Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Right
    private TrajectorySequence getRedRightRightTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo((RED_RIGHT_RIGHT_POSITION), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(RED_BACKDROP, Math.toRadians(180)))
//                .setReversed(false)
//                .splineTo(RED_MIDDLE, Math.toRadians(180))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE, Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(RED_MIDDLE, Math.toRadians(180))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(RED_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(RED_MIDDLE, Math.toRadians(0))
//                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    private TrajectorySequence getBlueLeftLeftTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(BLUE_LEFT_START, Math.toRadians(90));
        robotHardware.setPose(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(18)
                .setReversed(true)
                .splineTo(BLUE_LEFT_LEFT_RIGGING_POSITION, Math.toRadians(0))
                .turn(Math.toRadians(90))
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Middle
    private TrajectorySequence getBlueLeftMiddleTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(BLUE_LEFT_START, Math.toRadians(90));
        robotHardware.setPose(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(42)
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Right
    private TrajectorySequence getBlueLeftRightTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(BLUE_LEFT_START, Math.toRadians(90));
        robotHardware.setPose(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(BLUE_LEFT_RIGHT_SPIKE_MARK_POSITION, Math.toRadians(0)))
                .strafeLeft(20)
                .turn(Math.toRadians(-90))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Right

    // Right
    private TrajectorySequence getBlueRightLeftTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(BLUE_RIGHT_START, Math.toRadians(90));
        robotHardware.setPose(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo((BLUE_RIGHT_LEFT_POSITION), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(BLUE_BACKDROP, Math.toRadians(180)))
//                .setReversed(false)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Middle
    private TrajectorySequence getBlueRightMiddleTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(BLUE_RIGHT_START, Math.toRadians(90));
        robotHardware.setPose(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .lineToLinearHeading(new Pose2d(BLUE_BACKDROP, Math.toRadians(180)))
//                .setReversed(false)
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Right
    private TrajectorySequence getBlueRightRightTrajectorySequence(SampleMecanumDrive drive) {

        Pose2d startPose = new Pose2d(BLUE_RIGHT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(BLUE_RIGHT_RIGHT_POSITION, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(BLUE_BACKDROP, Math.toRadians(180)))
//                .setReversed(false)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
//                .setReversed(false)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
//                .splineTo(BLUE_PIXELS, Math.toRadians(180))
//                .setReversed(true)
//                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
//                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
//                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }
*/
    private void log(String message) {

        // If the telemetry is missing...
        if (telemetry == null) {

            // Exit the method.
            return;

        }

        // Show the message.
        telemetry.addData("Message", message);
        telemetry.update();

    }

    // Waits for menu selection.
    private void waitForMenuSelection() {

        // While the op mode is active...
        while (!isStopRequested()) {

            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // If the user has not selected an alliance...
            if (redAlliance == null) {
                telemetry.addData("Alliance", "X = blue, B = red");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    redAlliance = false;
                }
                if (currentGamepad.b && !previousGamepad.b) {
                    redAlliance = true;
                }
            }

            // Otherwise, if the user has not selected a starting location...
            else if (startLeft == null) {
                telemetry.addData("Start", "X = left, B = right");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    startLeft = true;
                }
                if (currentGamepad.b && !previousGamepad.b) {
                    startLeft = false;
                }
            }

            // Otherwise, if the user has not selected a parking location...
            else if (parkLeft == null) {
                telemetry.addData("Park", "X = left, B = right");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    parkLeft = true;
                }
                if (currentGamepad.b && !previousGamepad.b) {
                    parkLeft = false;
                }
            }

            // Otherwise (if the user finished making menu selections)...
            else {

                // Stop prompting the user for inputs.
                break;

            }

        }

    }

    // Waits for the camera to open.
    private void waitForCameraOpen() throws InterruptedException {

        // Verify inputs exist.
        if (hardwareMap == null) {
            throw new InterruptedException("The hardware map is missing.");
        }
        if (parkLeft == null) {
            throw new InterruptedException("The park left value is missing.");
        }
        if (redAlliance == null) {
            throw new InterruptedException("The red alliance value is missing.");
        }
        if (startLeft == null) {
            throw new InterruptedException("The start left value is missing.");
        }
        if (telemetry == null) {
            throw new InterruptedException("The telemetry is missing.");
        }

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Initialize the camera.
        log("Initializing camera...");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        teamPropDetector = new CenterStageCVDetection(parkLeft, redAlliance, startLeft, telemetry, true);
        camera.setPipeline(teamPropDetector);
        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.

        // Open the camera.
        log("Opening camera...");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                startedStreaming = true;
                log("Opened camera");
            }

            @Override
            public void onError(int errorCode) {
                log("Error opening camera: " + errorCode);
            }
        });

        // Wait for streaming to start.
        while (!isStopRequested() && !startedStreaming) {
            log("Waiting for camera to stream...");
            sleep(50);
        }

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Show the camera stream in the FTC dashboard.
        robotHardware.startCameraStream(camera);

    }

    // Waits for team prop detection.
    private void waitForTeamPropDetection() throws InterruptedException {

        // Verify inputs exist.
        if (camera == null) {
            throw new InterruptedException("The camera is missing.");
        }
        if (teamPropDetector == null) {
            throw new InterruptedException("The team prop detector is missing.");
        }

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Wait for team prop detection.
        while (!isStopRequested() && location == null) {
            log("Waiting for location detection...");
            sleep(50);
            location = teamPropDetector.getLocation();
        }

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Display the location.
        log("Location is " + location);

        // Stop streaming.
        camera.stopStreaming();

        // Close the camera.
        camera.closeCameraDevice();

    }

    // Sets the state.
    private void setState(State state) {

        // Set the state.
        this.state = state;

        // Reset the timer.
        timer.reset();

    }

    // Gets a start pose.
    private Pose2d getStartPose() throws InterruptedException {

        // Verify the inputs exist.
        if(redAlliance == null) {
            throw new InterruptedException("The red alliance value is missing.");
        }
        if(startLeft == null) {
            throw new InterruptedException("The start left value is missing.");
        }

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

    // Gets a spike mark trajectory sequence.
    private TrajectorySequence getSpikeMarkTrajectorySequence() throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Verify the inputs exist.
        if(drive == null) {
            throw new InterruptedException("The drive interface is missing.");
        }
        if(location == null) {
            throw new InterruptedException("The location is missing.");
        }
        if(redAlliance == null) {
            throw new InterruptedException("The red alliance value is missing.");
        }
        if(startLeft == null) {
            throw new InterruptedException("The start left value is missing.");
        }

        // Get a start pose.
        Pose2d startPose = getStartPose();

        // Set the robot's pose.
        robotHardware.setPose(startPose);

        // Set the trajectory sequence's start pose.
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(startPose);

        // Add the appropriate maneuvers.
        if (redAlliance) {
            if (startLeft) {
                if (location == location.Left) {
                    Pose2d targetPose = new Pose2d(RED_START_LEFT_SPIKE_LEFT_X, RED_START_LEFT_SPIKE_LEFT_Y, Math.toRadians(RED_START_LEFT_SPIKE_LEFT_HEADING));
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .lineToLinearHeading(targetPose);
                }
                else if (location == location.Middle) {
                    Pose2d targetPose = new Pose2d(RED_START_LEFT_SPIKE_MIDDLE_X, RED_START_LEFT_SPIKE_MIDDLE_Y, Math.toRadians(RED_START_LEFT_SPIKE_MIDDLE_HEADING));
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .lineToLinearHeading(targetPose);
                }
                else {
                    Pose2d targetPose = new Pose2d(RED_START_LEFT_SPIKE_RIGHT_X, RED_START_LEFT_SPIKE_RIGHT_Y);
                    double targetHeading = Math.toRadians(RED_START_LEFT_SPIKE_RIGHT_HEADING);
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .setReversed(true)
                            .splineToLinearHeading(targetPose, targetHeading);
                }
            }
            else {
                if (location == location.Left) {
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .splineToLinearHeading(new Pose2d(13,-30), Math.toRadians(180));
                }
                else if (location == location.Middle) {
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(12,-14, Math.toRadians(-90)));
                }
                else {
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .setReversed(true)
                            .lineToLinearHeading(new Pose2d(10,-30, Math.toRadians(0)));
                }
            }
        }
        else {
            if (startLeft) {
                if (location == location.Left) {
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(13,30, Math.toRadians(180)));
                } else if (location == location.Middle) {
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(12,14, Math.toRadians(90)));
                } else {
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(10,30), Math.toRadians(0));
                }
            } else {
                if (location == location.Left) {
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(-34,30, Math.toRadians(180)));
                } else if (location == location.Middle) {
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .lineToLinearHeading(new Pose2d(-36,14, Math.toRadians(90)));
                } else {
                    trajectorySequenceBuilder = trajectorySequenceBuilder
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-37,30),Math.toRadians(0));
                }
            }
        }

        // Build the trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the result.
        return trajectorySequence;

    }

        // Gets a backdrop trajectory sequence.
        private TrajectorySequence getBackdropTrajectorySequence() throws InterruptedException {

            // Get a drive interface.
            SampleMecanumDrive drive = robotHardware.getDrive();

            // Set the trajectory sequence's start pose.
            TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(lastEnd);

            Pose2d targetPose1 = new Pose2d(RED_MIDDLE_X, RED_MIDDLE_Y);
            double targetHeading1 = Math.toRadians(RED_TOWARDS_BACKDROP_HEADING);
            Pose2d targetPose2 = new Pose2d(RED_DETOUR_BACKDROP_X, RED_DETOUR_BACKDROP_Y);
            double targetHeading2 = Math.toRadians(RED_TOWARDS_BACKDROP_HEADING);
            Pose2d targetPose3 = new Pose2d(RED_BACKDROP_X, RED_BACKDROP_Y);
            double targetHeading3 = Math.toRadians(RED_TOWARDS_BACKDROP_HEADING);
            trajectorySequenceBuilder = trajectorySequenceBuilder
                    .setReversed(true)
                    .splineToLinearHeading(targetPose1, targetHeading1)
                    .splineToLinearHeading(targetPose2, targetHeading2)
                    .splineToLinearHeading(targetPose3, targetHeading3);

            // Add the appropriate maneuvers.
            /*if (redAlliance) {
                if (startLeft) {
                    if (location == location.Left) {
                        Pose2d targetPose = new Pose2d(RED_START_LEFT_SPIKE_LEFT_X, RED_START_LEFT_SPIKE_LEFT_Y, Math.toRadians(RED_START_LEFT_SPIKE_LEFT_HEADING));
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .lineToLinearHeading(targetPose);
                    }
                    else if (location == location.Middle) {
                        Pose2d targetPose = new Pose2d(RED_START_LEFT_SPIKE_MIDDLE_X, RED_START_LEFT_SPIKE_MIDDLE_Y, Math.toRadians(RED_START_LEFT_SPIKE_MIDDLE_HEADING));
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .lineToLinearHeading(targetPose);
                    }
                    else {
                        Pose2d targetPose = new Pose2d(RED_START_LEFT_SPIKE_RIGHT_X, RED_START_LEFT_SPIKE_RIGHT_Y);
                        double targetHeading = Math.toRadians(RED_START_LEFT_SPIKE_RIGHT_HEADING);
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .setReversed(true)
                                .splineToLinearHeading(targetPose, targetHeading);
                    }
                }
                else {
                    if (location == location.Left) {
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .splineToLinearHeading(new Pose2d(13,-30), Math.toRadians(180));
                    }
                    else if (location == location.Middle) {
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .lineToLinearHeading(new Pose2d(12,-14, Math.toRadians(-90)));
                    }
                    else {
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10,-30, Math.toRadians(0)));
                    }
                }
            }
            else {
                if (startLeft) {
                    if (location == location.Left) {
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .lineToLinearHeading(new Pose2d(13,30, Math.toRadians(180)));
                    } else if (location == location.Middle) {
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .lineToLinearHeading(new Pose2d(12,14, Math.toRadians(90)));
                    } else {
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(10,30), Math.toRadians(0));
                    }
                } else {
                    if (location == location.Left) {
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .lineToLinearHeading(new Pose2d(-34,30, Math.toRadians(180)));
                    } else if (location == location.Middle) {
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .lineToLinearHeading(new Pose2d(-36,14, Math.toRadians(90)));
                    } else {
                        trajectorySequenceBuilder = trajectorySequenceBuilder
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-37,30),Math.toRadians(0));
                    }
                }
            }*/

        // Build the trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the result.
        return trajectorySequence;

    }

}
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoF.State.IDLE;
import static org.firstinspires.ftc.teamcode.AutoF.State.STEP_A;
import static org.firstinspires.ftc.teamcode.AutoF.State.STEP_B;
import static org.firstinspires.ftc.teamcode.AutoF.State.STEP_C;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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
    public static final Vector2d RED_LEFT_START = new Vector2d(-36, -61);
    public static final Vector2d RED_RIGHT_START = new Vector2d(12, -61);
    public static final Vector2d RED_RIGHT_LEFT_POSITION = new Vector2d(14, -30);
    public static final Vector2d RED_RIGHT_RIGHT_POSITION = new Vector2d(23, -30);
    public static final Vector2d RED_LEFT_RIGHT_RIGGING_POSITION = new Vector2d(-9.5, -35);
    public static final Vector2d RED_LEFT_LEFT_SPIKE_MARK_POSITION = new Vector2d(-34, -34);
    public static final Vector2d BLUE_MIDDLE = new Vector2d(0, 12);
    public static final Vector2d BLUE_DETOUR_BACKDROP = new Vector2d(28, 12);
    public static final Vector2d BLUE_BACKDROP = new Vector2d(44, 36);
    public static final Vector2d BLUE_PIXELS = new Vector2d(-58, 10);
    public static final Vector2d BLUE_LEFT_START = new Vector2d(12, 61);
    public static final Vector2d BLUE_RIGHT_START = new Vector2d(-36, 61);
    public static final Vector2d BLUE_RIGHT_LEFT_POSITION = new Vector2d(23, 30);
    public static final Vector2d BLUE_RIGHT_RIGHT_POSITION = new Vector2d(14, 30);
    public static final Vector2d BLUE_LEFT_LEFT_RIGGING_POSITION = new Vector2d(-9.5, 35);
    public static final Vector2d BLUE_LEFT_RIGHT_SPIKE_MARK_POSITION = new Vector2d(-34, 34);
    public static final Route ROUTE = Route.RED_LEFT_DIRECT;
    public static final double DELAY = 0.5;
    private static final String TAG = "Bucket Brigade";
    public static final int FIRST_ROW = 1;
    public static double PURPLE_PIXEL_DELAY = 1;
    public static Boolean redAlliance;
    public static Pose2d currentPose;
    public static Boolean lastRanAutonomous;
    private Boolean startLeft;
    private Boolean parkLeft;
    private OpenCvWebcam camera;
    private boolean startedStreaming;
    private Gamepad previousGamepad = new Gamepad();
    private Gamepad currentGamepad = new Gamepad();
    private CenterStageCVDetection.Location location;
    private CenterStageCVDetection teamPropDetector;
    private RobotHardwareC robotHardware;
    enum State { IDLE, STEP_A, STEP_B, STEP_C, STEP_D, STEP_E, STEP_F, STEP_G, STEP_H, STEP_I, STEP_J, STEP_K, STEP_L, STEP_M, STEP_N }
    private State state = IDLE;
    private ElapsedTime timer = new ElapsedTime();

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

        // Raise the wrist.
        robotHardware.raiseWrist();

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

        /*if (redAlliance) {
            if (startLeft) {
                if (location == location.Left) {
                    sequence = getRedLeftLeftTrajectorySequence(drive);
                }
                if (location == location.Middle) {
                    sequence = getRedLeftMiddleTrajectorySequence(drive);
                }
                if (location == location.Right) {
                    sequence = getRedLeftRightTrajectorySequence(drive);
                }
            }
            else {
                if (location == location.Left) {
                    sequence = getRedRightLeftTrajectorySequence(drive);
                }
                if (location == location.Middle) {
                    sequence = getRedRightMiddleTrajectorySequence(drive);
                }
                if (location == location.Right) {
                    sequence = getRedRightRightTrajectorySequence(drive);
                }
            }
        }
        else {
            if (startLeft) {
                if (location == location.Left) {
                    sequence = getBlueLeftLeftTrajectorySequence(drive);
                }
                if (location == location.Middle) {
                    sequence = getBlueLeftMiddleTrajectorySequence(drive);
                }
                if (location == location.Right) {
                    sequence = getBlueLeftRightTrajectorySequence(drive);
                }
            }
            else {
                if (location == location.Left) {
                    sequence = getBlueRightLeftTrajectorySequence(drive);
                }
                if (location == location.Middle) {
                    sequence = getBlueRightMiddleTrajectorySequence(drive);
                }
                if (location == location.Right) {
                    sequence = getBlueRightRightTrajectorySequence(drive);
                }
            }
        }*/

        // Initialize the drive interface.
        robotHardware.initializeDrive();

        // Start the state machine.
        setState(STEP_A);

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

            case STEP_A:

                TrajectorySequence trajectorySequence;

                // Get a trajectory sequence.
                //trajectorySequence = getRedLeftMiddleSpikeMarkTrajectorySequence(drive);

                if (redAlliance) {
                    if (startLeft) {
                        if (location == location.Left) {
                            trajectorySequence = getRedLeftLeftSpikeMarkTrajectorySequence(drive);
                        }
                        else if (location == location.Middle) {
                            trajectorySequence = getRedLeftMiddleSpikeMarkTrajectorySequence(drive);
                        }
                        else if (location == location.Right) {
                            trajectorySequence = getRedLeftRightSpikeMarkTrajectorySequence(drive);
                        }
                        else {
                            throw new InterruptedException("Location is not recognized");
                        }
                    }
                    else {
                        if (location == location.Left) {
                            trajectorySequence = getRedRightLeftSpikeMarkTrajectorySequence(drive);
                        }
                        else if (location == location.Middle) {
                            trajectorySequence = getRedRightMiddleSpikeMarkTrajectorySequence(drive);
                        }
                        else if (location == location.Right) {
                            trajectorySequence = getRedRightRightSpikeMarkTrajectorySequence(drive);
                        }
                        else {
                            throw new InterruptedException("Location is not recognized");
                        }
                    }
                }
                else {
                    if (startLeft) {
                        if (location == location.Left) {
                            trajectorySequence = getBlueLeftLeftSpikeMarkTrajectorySequence(drive);
                        } else if (location == location.Middle) {
                            trajectorySequence = getBlueLeftMiddleSpikeMarkTrajectorySequence(drive);
                        } else if (location == location.Right) {
                            trajectorySequence = getBlueLeftRightSpikeMarkTrajectorySequence(drive);
                        } else {
                            throw new InterruptedException("Location is not recognized");
                        }
                    } else {
                        if (location == location.Left) {
                            trajectorySequence = getBlueRightLeftSpikeMarkTrajectorySequence(drive);
                        } else if (location == location.Middle) {
                            trajectorySequence = getBlueRightMiddleSpikeMarkTrajectorySequence(drive);
                        } else if (location == location.Right) {
                            trajectorySequence = getBlueRightRightSpikeMarkTrajectorySequence(drive);
                        } else {
                            throw new InterruptedException("Location is not recognized");
                        }
                    }
                }

                // Start following the trajectory sequence.
                drive.followTrajectorySequenceAsync(trajectorySequence);

                // Advance to step B.
                setState(STEP_B);

                break;

            case STEP_B:

                // If we are waiting...
                if (timer.milliseconds() < 1000) {

                    // Exit the method.
                    return;

                }

                // Lower the wrist.
                robotHardware.lowerWrist();

                // Advance to step C.
                setState(STEP_C);

                break;

            case STEP_C:

                // If the robot is driving...
                if (drive.isBusy()) {

                    // Exit the method.
                    return;

                }

                // Open the left claw.
                robotHardware.openLeftClawFully();

                // Advance to the idle step.
                setState(IDLE);

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

    private TrajectorySequence getRedLeftLeftSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_LEFT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-34,-30,Math.toRadians(180)))
                .build();
        return sequence;
    }

    private TrajectorySequence getRedLeftMiddleSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_LEFT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36,-14, Math.toRadians(-90)))
                .build();
        return sequence;
    }

    private TrajectorySequence getRedLeftRightSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_LEFT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-37,-30),Math.toRadians(0))
                .build();
        return sequence;
    }

    private TrajectorySequence getRedRightLeftSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(13,-30), Math.toRadians(180))
                .build();
        return sequence;
    }

    private TrajectorySequence getRedRightMiddleSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,-14, Math.toRadians(-90)))
                .build();
        return sequence;
    }

    private TrajectorySequence getRedRightRightSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(10,-30,Math.toRadians(0)))
                .build();
        return sequence;
    }

    private TrajectorySequence getBlueLeftLeftSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_LEFT_START, Math.toRadians(90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(13,30,Math.toRadians(180)))
                .build();
        return sequence;
    }

    private TrajectorySequence getBlueLeftMiddleSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_LEFT_START, Math.toRadians(90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,14, Math.toRadians(90)))
                .build();
        return sequence;
    }

    private TrajectorySequence getBlueLeftRightSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_LEFT_START, Math.toRadians(90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(10,30),Math.toRadians(0))
                .build();
        return sequence;
    }

    private TrajectorySequence getBlueRightLeftSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_RIGHT_START, Math.toRadians(90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-34,30,Math.toRadians(180)))
                .build();
        return sequence;
    }

    private TrajectorySequence getBlueRightMiddleSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_RIGHT_START, Math.toRadians(90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36,14, Math.toRadians(90)))
                .build();
        return sequence;
    }

    private TrajectorySequence getBlueRightRightSpikeMarkTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_RIGHT_START, Math.toRadians(90));
        robotHardware.setPose(startPose);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-37,30),Math.toRadians(0))
                .build();
        return sequence;
    }

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
//                .addTemporalMarker(PURPLE_PIXEL_DELAY, () -> {
//                    robotHardware.openLeftClaw();
//                })
                .back(42)
                .setReversed(true)
//                .UNSTABLE_addDisplacementMarkerOffset(4, () -> {
//                    robotHardware.openLeftClaw();
//                })
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
//                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
//                    robotHardware.raiseArm();
//
//                    int targetLiftPosition = HeatSeekC.getTargetLiftPosition(FIRST_ROW);
//                    robotHardware.raiseLift(targetLiftPosition);
//                    robotHardware.openRightClaw();
//                })
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
}
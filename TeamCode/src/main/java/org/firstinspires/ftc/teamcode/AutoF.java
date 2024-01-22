package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoG.getDetection;
import static org.firstinspires.ftc.teamcode.AutoG.getRobotPose;
import static org.firstinspires.ftc.teamcode.AutoG.isMatch;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Config
@Autonomous
//@Autonomous(preselectTeleOp = "TeleOpR")
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
    private Boolean redAlliance = null;
    private Boolean startLeft = null;
    private Boolean parkLeft = null;
    private OpenCvWebcam camera;
    private boolean startedStreaming;
    private AprilTagProcessor aprilTagProcessor;
    private Gamepad previousGamepad = new Gamepad();
    private Gamepad currentGamepad = new Gamepad();
    private FtcDashboard ftcDashboard;
    private CenterStageCVDetection.Location location;
    private CenterStageCVDetection teamPropDetector;

    @Override
    public void runOpMode() throws InterruptedException {

        // Get an FTC dashboard.
        ftcDashboard = FtcDashboard.getInstance();

        // Construct a drive interface.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Wait for menu selection.
        waitForMenuSelection();

        // Wait for the camera to open.
        waitForCameraOpen();

        // Wait for start.
        waitForStart();

        // Wait for team prop detection.
        waitForTeamPropDetection();

        // Construct an AprilTag processor.
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Construct a vision portal.
        new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT))
                .addProcessor(aprilTagProcessor)
                .build();

        //TrajectorySequence sequence = null;

        TrajectorySequence sequence = getRedLeftMiddleTrajectorySequence(drive);

//        if (redAlliance) {
//            if (startLeft) {
//                if (location == location.Left) {
//                    sequence = getRedLeftLeftTrajectorySequence(drive);
//                }
//                if (location == location.Middle) {
//                    sequence = getRedLeftMiddleTrajectorySequence(drive);
//                }
//                if (location == location.Right) {
//                    sequence = getRedLeftRightTrajectorySequence(drive);
//                }
//            }
//            else {
//                if (location == location.Left) {
//                    sequence = getRedRightLeftTrajectorySequence(drive);
//                }
//                if (location == location.Middle) {
//                    sequence = getRedRightMiddleTrajectorySequence(drive);
//                }
//                if (location == location.Right) {
//                    sequence = getRedRightRightTrajectorySequence(drive);
//                }
//            }
//        }
//        else {
//            if (startLeft) {
//                if (location == location.Left) {
//                    sequence = getBlueLeftLeftTrajectorySequence(drive);
//                }
//                if (location == location.Middle) {
//                    sequence = getBlueLeftMiddleTrajectorySequence(drive);
//                }
//                if (location == location.Right) {
//                    sequence = getBlueLeftRightTrajectorySequence(drive);
//                }
//            }
//            else {
//                if (location == location.Left) {
//                    sequence = getBlueRightLeftTrajectorySequence(drive);
//                }
//                if (location == location.Middle) {
//                    sequence = getBlueRightMiddleTrajectorySequence(drive);
//                }
//                if (location == location.Right) {
//                    sequence = getBlueRightRightTrajectorySequence(drive);
//                }
//            }
//        }
//
//        if (sequence == null) {
//            throw new InterruptedException("The sequence is missing.");
//        }

        drive.followTrajectorySequenceAsync(sequence);
        //drive.followTrajectorySequence(sequence);

        // Initialize a localized value.
        boolean localized = false;

        // While the op mode is active...
        while (!isStopRequested()) {

            // Update the robot.
            drive.update();

            // Get a detection.
            AprilTagDetection detection = AutoG.getDetection(aprilTagProcessor);

            // If there is a detection...
            if (detection != null) {

                // Get the robot's pose.
                Pose2d pose = AutoG.getRobotPose(detection, telemetry);

                // Update the driver interface.
                drive.setPoseEstimate(pose);

                // Remember that we localized the robot using an AprilTag.
                localized = true;

            }

            // Get the robot's pose.
            Pose2d pose = drive.getPoseEstimate();

            // Convert the pose to a string.
            String poseString = toString(pose);

            // Update the telemetry.
            telemetry.addData("Localized", localized);
            telemetry.addData("Pose", poseString);
            telemetry.update();

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

    private TrajectorySequence getRedLeftLeftTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_LEFT_START, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(RED_LEFT_LEFT_SPIKE_MARK_POSITION, Math.toRadians(0)))
                .strafeLeft(20)
                .turn(Math.toRadians(-90))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_MIDDLE, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_MIDDLE, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Middle
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

    // Right
    private TrajectorySequence getRedLeftRightTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_LEFT_START, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(18)
                .setReversed(true)
                .splineTo(RED_LEFT_RIGHT_RIGGING_POSITION, Math.toRadians(0))
                .turn(Math.toRadians(90))
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_MIDDLE, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_MIDDLE, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Right

    // Left
    private TrajectorySequence getRedRightLeftTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(RED_RIGHT_LEFT_POSITION, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(RED_BACKDROP, Math.toRadians(180)))
                .setReversed(false)
                .splineTo(RED_MIDDLE, Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_MIDDLE, Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Middle
    private TrajectorySequence getRedRightMiddleTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .lineToLinearHeading(new Pose2d(RED_BACKDROP, Math.toRadians(180)))
                .setReversed(false)
                .splineTo(RED_MIDDLE, Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_MIDDLE, Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Right
    private TrajectorySequence getRedRightRightTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo((RED_RIGHT_RIGHT_POSITION), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(RED_BACKDROP, Math.toRadians(180)))
                .setReversed(false)
                .splineTo(RED_MIDDLE, Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_MIDDLE, Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE, Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

// Blue

    // Left

    // Left
    private TrajectorySequence getBlueLeftLeftTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_LEFT_START, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(18)
                .setReversed(true)
                .splineTo(BLUE_LEFT_LEFT_RIGGING_POSITION, Math.toRadians(0))
                .turn(Math.toRadians(90))
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Middle
    private TrajectorySequence getBlueLeftMiddleTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_LEFT_START, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(32)
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Right
    private TrajectorySequence getBlueLeftRightTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_LEFT_START, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(BLUE_LEFT_RIGHT_SPIKE_MARK_POSITION, Math.toRadians(0)))
                .strafeLeft(20)
                .turn(Math.toRadians(-90))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Right

    // Right
    private TrajectorySequence getBlueRightLeftTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_RIGHT_START, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo((BLUE_RIGHT_LEFT_POSITION), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(BLUE_BACKDROP, Math.toRadians(180)))
                .setReversed(false)
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Middle
    private TrajectorySequence getBlueRightMiddleTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_RIGHT_START, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .lineToLinearHeading(new Pose2d(BLUE_BACKDROP, Math.toRadians(180)))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    // Right
    private TrajectorySequence getBlueRightRightTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(BLUE_RIGHT_START, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(BLUE_RIGHT_RIGHT_POSITION, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(BLUE_BACKDROP, Math.toRadians(180)))
                .setReversed(false)
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_MIDDLE, Math.toRadians(180))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE, Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .build();
        return sequence;
    }

    private void log(String message) {
        telemetry.addData("Message", message);
        telemetry.update();
    }

    private void waitForMenuSelection() {
        while (!isStopRequested()) {
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
    }

    private void waitForCameraOpen() throws InterruptedException {

        // Verify inputs exist.
        if (ftcDashboard == null) {
            throw new InterruptedException("The FTC dashboard is missing.");
        }
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

        // Initialize the camera.
        log("initializing camera");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        teamPropDetector = new CenterStageCVDetection(parkLeft, redAlliance, startLeft, telemetry);
        camera.setPipeline(teamPropDetector);
        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.

        // Open the camera.
        log("opening camera");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                startedStreaming = true;
                log("opened camera");
            }

            @Override
            public void onError(int errorCode) {
                log("error opening camera: " + errorCode);
            }
        });

        // Wait for streaming to start.
        while (!isStopRequested() && !startedStreaming) {
            log("waiting for camera to start streaming");
            sleep(50);
        }

        // Show the camera stream in the FTC dashboard.
        ftcDashboard.startCameraStream(camera, 0);

    }

    private void waitForTeamPropDetection() throws InterruptedException {

        // Verify inputs exist.
        if (camera == null) {
            throw new InterruptedException("The camera is missing.");
        }
        if (teamPropDetector == null) {
            throw new InterruptedException("The team prop detector is missing.");
        }

        // Wait for team prop detection.
        while (!isStopRequested() && location == null) {
            log("waiting for location detection");
            sleep(50);
            location = teamPropDetector.getLocation();
        }

        // Display the location.
        log("location is " + location);

        // Stop streaming.
        camera.stopStreaming();

        // Close the camera.
        camera.closeCameraDevice();

    }
}
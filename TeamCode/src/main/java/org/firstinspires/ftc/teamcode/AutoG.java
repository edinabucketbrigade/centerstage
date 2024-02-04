package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp = "TeleOpA")
@Config
@Disabled
public class AutoG extends LinearOpMode {

    // Tag identifiers
    private static final int TAG_ID_BLUE_ALLIANCE_BACKDROP_LEFT = 1;
    private static final int TAG_ID_BLUE_ALLIANCE_BACKDROP_MIDDLE = 2;
    private static final int TAG_ID_BLUE_ALLIANCE_BACKDROP_RIGHT = 3;
    private static final int TAG_ID_RED_ALLIANCE_BACKDROP_LEFT = 4;
    private static final int TAG_ID_RED_ALLIANCE_BACKDROP_MIDDLE = 5;
    private static final int TAG_ID_RED_ALLIANCE_BACKDROP_RIGHT = 6;
    private static final int TAG_ID_RED_ALLIANCE_WALL_LARGE = 7;
    private static final int TAG_ID_RED_ALLIANCE_WALL_SMALL = 8;
    private static final int TAG_ID_BLUE_ALLIANCE_WALL_SMALL = 9;
    private static final int TAG_ID_BLUE_ALLIANCE_WALL_LARGE = 10;

    // Y offset between the camera's front the the robot's center
    private static final double CAMERA_Y_OFFSET = 7;

    // Maximum pitch for valid AprilTag detection
    public static final double MAXIMUM_PITCH = 10;

    // Maximum velocity
    private static final double MAXIMUM_VELOCITY = DriveConstants.MAX_VEL;

    // Maximum acceleration
    private static final double MAXIMUM_ACCELERATION = DriveConstants.MAX_ACCEL;

    // Used to detect april tags
    private AprilTagProcessor aprilTagProcessor;

    // Runs the op mode.
    @Override
    public void runOpMode() {

        // Initialize the FTC dashboard.
        FtcDashboard.getInstance();

        // Update the telemetry.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Get an AprilTag processor.
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Get a vision portal.
        new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT))
                .addProcessor(aprilTagProcessor)
                .build();

        // Get a drive.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Wait for the user to press start.
        waitForStart();

        // Wait for an AprilTag detection.
        AprilTagDetection detection = waitForDetection();

        // Get the robot's pose.
        Pose2d startPose = getRobotPose(detection, telemetry);

        // Update the telemetry.
        telemetry.update();

        // Set the drive's pose estimate.
        drive.setPoseEstimate(startPose);

        // Construct a target pose.
        Pose2d targetPose = new Pose2d(-58, 35, Math.toRadians(0)); // blue wall small
        //Pose2d targetPose = new Pose2d(-58, -35, Math.toRadians(0)); // red wall small
        //Pose2d targetPose = new Pose2d(50, 35, Math.toRadians(180)); // blue backdrop middle
        //Pose2d targetPose = new Pose2d(50, -35, Math.toRadians(180)); // red backdrop middle

        // Construct a velocity constraint.
        TrajectoryVelocityConstraint velocityConstraint = new MecanumVelocityConstraint(MAXIMUM_VELOCITY, DriveConstants.TRACK_WIDTH);

        // Construct an acceleration constraint.
        TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(MAXIMUM_ACCELERATION);

        // Construct a trajectory sequence.
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(velocityConstraint, accelerationConstraint)
                .lineToLinearHeading(targetPose)
                .build();

        // Execute the trajectory sequence.
        drive.followTrajectorySequence(sequence);

        // Wait for the user to press the stop.
        waitForStop();

    }

    // Waits for an AprilTag detection.
    private AprilTagDetection waitForDetection() {

        // While the op mode is active...
        while (opModeIsActive()) {

            // Indicate that we are looking for an AprilTag.
            telemetry.addData("Status", "Looking for AprilTag...");

            // Update the telemetry.
            telemetry.update();

            // Get a detection.
            AprilTagDetection detection = getDetection(aprilTagProcessor);

            // If the detection is missing...
            if (detection == null) {

                // Skip this detection.
                continue;

            }

            // Return the detection.
            return detection;

        }

        // Exit the method.
        return null;

    }

    // Offsets a point by a given distance and angle.
    private static Vector2d offset(Vector2d inputPoint, double distance, double degrees) {

        // Get the input coordinates.
        double inputX = inputPoint.getX();
        double inputY = inputPoint.getY();

        // Convert the angle from degrees to radians.
        double radians = Math.toRadians(degrees);

        // Get output coordinates.
        double outputX = inputX + distance * Math.cos(radians);
        double outputY = inputY - distance * Math.sin(radians);

        // Construct an output point.
        Vector2d outputPoint = new Vector2d(outputX, outputY);

        // Return the result.
        return outputPoint;

    }

    // Determines whether an AprilTag detection is a match.
    public static boolean isMatch(AprilTagDetection detection) {

        // If the detection's metdata is missing...
        if (detection.metadata == null) {

            // Return indicating that the detection is not a match.
            return false;

        }

        // Get the detection's pose.
        AprilTagPoseFtc pose = detection.ftcPose;

        // Get the pose's pitch.
        double pitch = pose.pitch;

        // If the pitch is invalid...
        if (pitch > MAXIMUM_PITCH) {

            // Return indicating that the detection is not a match.
            return false;

        }
        // Get the tag identifier.
        int tagId = detection.id;

        // Determine whether the detection is a match.
        //boolean isMatch = isBackdropTag(tagId) || isWallTag(tagId);
        boolean isMatch = isBackdropTag(tagId);

        // Return the result.
        return isMatch;

    }

    // Gets an AprilTag detection.
    public static AprilTagDetection getDetection(AprilTagProcessor aprilTagProcessor) {

        // Get the AprilTag detections.
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        // For each detection...
        for (AprilTagDetection detection : detections) {

            // If the detection is a match...
            if (isMatch(detection)) {

                // Return the detection.
                return detection;

            }

        }

        // If we made it this far, return indicating that there are no detections.
        return null;

    }

    // Gets the robot's pose from an AprilTag detection.
    public static Pose2d getRobotPose(AprilTagDetection detection, Telemetry telemetry) {

        // Get the tag's pose.
        AprilTagPoseFtc tagPose = detection.ftcPose;

        // Get the tag's metadata.
        AprilTagMetadata tagMetadata = detection.metadata;

        // Get the tag's field position.
        VectorF fieldPosition = tagMetadata.fieldPosition;

        // Get the tag's field orientation.
        Quaternion fieldOrientation = tagMetadata.fieldOrientation;

        // Get the tag identifier.
        int tagId = detection.id;

        // Get a tag point.
        double tagX = fieldPosition.get(0);
        double tagY = fieldPosition.get(1);
        Vector2d tagPoint = new Vector2d(tagX, tagY);

        // Get the pose's coordinates.
        double poseX = tagPose.x;
        double poseY = tagPose.y;

        // Get the pose's yaw.
        double poseYaw = tagPose.yaw;

        // Determine whether this is a wall tag.
        boolean isWallTag = isWallTag(tagId);

        // Get the camera's location by offsetting from the tag's location.
        Vector2d cameraPoint = offset(tagPoint, isWallTag ? poseY : -poseY, poseYaw);
        cameraPoint = offset(cameraPoint, isWallTag ? poseX : -poseX, 90 + poseYaw);

        // Get the camera's coordinates.
        double cameraX = cameraPoint.getX();
        double cameraY = cameraPoint.getY();

        // Get the camera's heading.
        double cameraHeading = isWallTag ? -poseYaw + 180 : -poseYaw;

        // Get the robot's center location by offsetting from the camera's location.
        Vector2d robotPoint = offset(cameraPoint, isWallTag ? CAMERA_Y_OFFSET : -CAMERA_Y_OFFSET, poseYaw);

        // Get the robot's coordinates.
        double robotX = robotPoint.getX();
        double robotY = robotPoint.getY();

        // Get the robot's heading.
        double robotHeading = cameraHeading - 180;

        // Get the robot's pose.
        Pose2d robotPose = new Pose2d(robotX, robotY, Math.toRadians(robotHeading));

        // Update the telemetry.
        telemetry.addData("Tag ID", tagId);
        telemetry.addData("Tag Position", fieldPosition);
        telemetry.addData("Tag Orientation", fieldOrientation);
        telemetry.addData("Tag Pose", "x %.2f, y %.2f, z %.2f, bearing %.2f, yaw %.2f, pitch %.2f, elevation %.2f, range %.2f, roll %.2f", tagPose.x, tagPose.y, tagPose.z, tagPose.bearing, tagPose.yaw, tagPose.pitch, tagPose.elevation, tagPose.range, tagPose.roll);
        telemetry.addData("camera", "x %.2f, y %.2f, heading %.2f", cameraX, cameraY, cameraHeading);
        telemetry.addData("robot", "x %.2f, y %.2f, heading %.2f", robotX, robotY, robotHeading);

        // Return the robot's pose.
        return robotPose;

    }

    // Waits for the user to press stop.
    private void waitForStop() {

        // Wait for the user to press stop.
        while (opModeIsActive()) {}

    }

    // Determines whether this is a backdrop tag.
    private static boolean isBackdropTag(int tagId) {

        // Determine whether this is a backdrop tag.
        boolean isBackdropTag =
                tagId == TAG_ID_BLUE_ALLIANCE_BACKDROP_LEFT ||
                tagId == TAG_ID_BLUE_ALLIANCE_BACKDROP_MIDDLE ||
                tagId == TAG_ID_BLUE_ALLIANCE_BACKDROP_RIGHT ||
                tagId == TAG_ID_RED_ALLIANCE_BACKDROP_LEFT ||
                tagId == TAG_ID_RED_ALLIANCE_BACKDROP_MIDDLE ||
                tagId == TAG_ID_RED_ALLIANCE_BACKDROP_RIGHT;

        // Return the result.
        return isBackdropTag;

    }

    // Determines whether this is a wall tag.
    private static boolean isWallTag(int tagId) {

        // Determine whether this is a wall tag.
        boolean isWallTag =
                tagId == TAG_ID_RED_ALLIANCE_WALL_LARGE ||
                tagId == TAG_ID_RED_ALLIANCE_WALL_SMALL ||
                tagId == TAG_ID_BLUE_ALLIANCE_WALL_SMALL ||
                tagId == TAG_ID_BLUE_ALLIANCE_WALL_LARGE;

        // Return the result.
        return isWallTag;

    }

}
package org.firstinspires.ftc.teamcode;

import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_ACCELERATION;
import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_VELOCITY_NORMAL;
import static bucketbrigade.casperlibrary.RobotRoutes.TRACK_WIDTH;

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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.LinkedList;
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

    // Number of AprilTags to consider when looking for an accurate detection
    public static int TAG_WINDOW = 5;

    // Heading threshold in degrees
    public static double HEADING_THRESHOLD = 5;

    // Position threshold in inches
    public static double POSITION_THRESHOLD = 2;
    public static boolean SHOW_TELEMETRY = false;

    // Recent AprilTag detections
    private static LinkedList<AprilTagDetection> recentDetections = new LinkedList<>();

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

        // Wait for a robot pose.
        Pose2d startPose = waitForRobotPose();

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
        TrajectoryVelocityConstraint velocityConstraint = new MecanumVelocityConstraint(MAXIMUM_VELOCITY_NORMAL, TRACK_WIDTH);

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

    // Waits for a robot pose.
    private Pose2d waitForRobotPose() {

        // While the op mode is active...
        while (opModeIsActive()) {

            // Indicate that we are looking for an AprilTag.
            telemetry.addData("Status", "Looking for AprilTag...");

            // Update the telemetry.
            telemetry.update();

            // Get a robot pose.
            Pose2d pose = getRobotPose(aprilTagProcessor, telemetry);

            // If the pose is missing...
            if (pose == null) {

                // Skip this pose.
                continue;

            }

            // Return the pose.
            return pose;

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

        // Get the tag identifier.
        int tagId = detection.id;

        // Determine whether the detection is a match.
        boolean isMatch = isBackdropTag(tagId) || isWallTag(tagId);

        // Return the result.
        return isMatch;

    }

    // Gets a robot pose from recent AprilTag detections.
    public static Pose2d getRobotPose(AprilTagProcessor aprilTagProcessor, Telemetry telemetry) {

        // Update the recent AprilTag detection list.
        //////////////////////////////////////////////////////////////////////

        // Get the current AprilTag detections.
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        // Get the matching AprilTag detections.
        List<AprilTagDetection> matchingDetections = getMatches(currentDetections);

        // For each matching detection...
        for(AprilTagDetection detection : matchingDetections) {

            // Count the recent detections.
            int recentCount = recentDetections.size();

            // If we have too many recent detections...
            if(recentCount >= TAG_WINDOW) {

                // Delete the oldest recent detection.
                recentDetections.pollFirst();

            }

            // Add the matching detection to the recent detection list.
            recentDetections.addLast(detection);

        }

        // Verify that we have enough recent AprilTag detections.
        //////////////////////////////////////////////////////////////////////

        // Count the matching detections.
        int matchingCount = matchingDetections.size();

        // Count the recent detections.
        int recentCount = recentDetections.size();

        // Display the detection counts.
        if(SHOW_TELEMETRY) {
            telemetry.addData("Detections", "matching = %d, recent = %d", matchingCount, recentCount);
        }

        // If we do not have enough recent detections...
        if(recentCount < TAG_WINDOW) {

            // Return indicating that we cannot determine the robot's pose.
            return null;

        }

        // Convert the recent AprilTag detections to robot poses.
        //////////////////////////////////////////////////////////////////////

        // Construct a pose list.
        List<Pose2d> recentPoses = new ArrayList<>();

        // For each recent detection...
        for(AprilTagDetection detection : recentDetections) {

            // Get a robot pose.
            Pose2d recentPose = getRobotPose(detection, telemetry);

            // Add the pose to the list.
            recentPoses.add(recentPose);

        }

        // Clear the recent detections.
        //////////////////////////////////////////////////////////////////////

        // Clear the recent detections.
        recentDetections.clear();

        // Get the average pose.
        //////////////////////////////////////////////////////////////////////

        // Get the average pose.
        Pose2d averagePose = getAveragePose(recentPoses);

        // Verify the recent poses are close to the average.
        //////////////////////////////////////////////////////////////////////

        // Get the average pose values.
        double averageHeadingRadians = averagePose.getHeading();
        double averageHeadingDegrees = Math.toDegrees(averageHeadingRadians);
        double averageX = averagePose.getX();
        double averageY = averagePose.getY();

        // Display the average pose.
        if(SHOW_TELEMETRY) {
            telemetry.addData("Average Pose", "x = %.1f, y = %.1f, heading = %.1f", averageX, averageY, averageHeadingDegrees);
        }

        // For each recent pose...
        for(Pose2d recentPose : recentPoses) {

            // Get the recent pose values.
            double recentHeadingRadians = recentPose.getHeading();
            double recentHeadingDegrees = Math.toDegrees(recentHeadingRadians);
            double recentX = recentPose.getX();
            double recentY = recentPose.getY();

            // Determine whether the average and recent poses match.
            boolean headingMatch = areEqual(averageHeadingDegrees, recentHeadingDegrees, HEADING_THRESHOLD);
            boolean xMatch = areEqual(averageX, recentX, POSITION_THRESHOLD);
            boolean yMatch = areEqual(averageY, recentY, POSITION_THRESHOLD);
            boolean isMatch = headingMatch && xMatch && yMatch;

            // Display the recent pose.
            if(SHOW_TELEMETRY) {
                telemetry.addData("Recent Pose", "x = %.1f, y = %.1f, heading = %.1f, match = %b", recentX, recentY, recentHeadingDegrees, isMatch);
            }

            // If the poses do not match...
            if(!isMatch) {

                // Return indicating that we cannot determine the robot's pose.
                return null;

            }

        }

        // Return the average of the recent poses.
        //////////////////////////////////////////////////////////////////////

        // Return the average of the recent poses.
        return averagePose;

    }

    // Get matching AprilTag detections.
    public static List<AprilTagDetection> getMatches(List<AprilTagDetection> inputDetections) {

        // Initialize an output list.
        List<AprilTagDetection> outputDetections = new ArrayList<>();

        // For each input detection...
        for (AprilTagDetection detection : inputDetections) {

            // If the detection is a match...
            if (isMatch(detection)) {

                // Add the detection to the output list.
                outputDetections.add(detection);

            }

        }

        // Return the output detections.
        return outputDetections;

    }

    // Gets the robot's pose from an AprilTag detection.
    private static Pose2d getRobotPose(AprilTagDetection detection, Telemetry telemetry) {

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
        if(SHOW_TELEMETRY) {
            telemetry.addData("Tag ID", tagId);
            telemetry.addData("Tag Position", fieldPosition);
            telemetry.addData("Tag Orientation", fieldOrientation);
            telemetry.addData("Tag Pose", "x %.2f, y %.2f, z %.2f, bearing %.2f, yaw %.2f, pitch %.2f, elevation %.2f, range %.2f, roll %.2f", tagPose.x, tagPose.y, tagPose.z, tagPose.bearing, tagPose.yaw, tagPose.pitch, tagPose.elevation, tagPose.range, tagPose.roll);
            telemetry.addData("camera", "x %.2f, y %.2f, heading %.2f", cameraX, cameraY, cameraHeading);
            telemetry.addData("robot", "x %.2f, y %.2f, heading %.2f", robotX, robotY, robotHeading);
        }

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

    // Determines whether values are equal within a threshold.
    public static boolean areEqual(double a, double b, double threshold) {
        return Math.abs(a - b) <= threshold;
    }

    // Gets an average pose.
    public static Pose2d getAveragePose(List<Pose2d> inputPoses) {

        // Construct value lists.
        List<Double> headings = new ArrayList<>();
        List<Double> xCoordinates = new ArrayList<>();
        List<Double> yCoordinates = new ArrayList<>();

        // For each pose...
        for(Pose2d inputPose : inputPoses) {

            // Get the input pose values.
            double inputHeading = inputPose.getHeading();
            double inputX = inputPose.getX();
            double inputY = inputPose.getY();

            // Add the values to the lists.
            headings.add(inputHeading);
            xCoordinates.add(inputX);
            yCoordinates.add(inputY);

        }

        // Get the average pose.
        double averageHeading = getAverageValue(headings);
        double averageX = getAverageValue(xCoordinates);
        double averageY = getAverageValue(yCoordinates);
        Pose2d averagePose = new Pose2d(averageX, averageY, averageHeading);

        // Return the average pose.
        return averagePose;

    }

    // Gets an average.
    public static double getAverageValue(List<Double> values) {
        return values.stream().mapToDouble(val -> val).average().orElse(0.0);
    }

}
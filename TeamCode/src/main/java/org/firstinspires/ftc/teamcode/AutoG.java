package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp = "TeleOpA")
@Config
//@Disabled
public class AutoG extends LinearOpMode {
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection process.
    private HeatSeek.TargetMode targetMode = HeatSeek.TargetMode.NONE;
    private boolean initialized;

    @Override
    public void runOpMode() {
        FtcDashboard.getInstance();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT))
                .addProcessor(aprilTag)
                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        AprilTagDetection detection = null;

        while (opModeIsActive() && detection == null) {
            detection = getDetection();
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

        AprilTagMetadata tagMetadata = detection.metadata;
        VectorF fieldPosition = tagMetadata.fieldPosition;
        Quaternion fieldOrientation = tagMetadata.fieldOrientation;
        AprilTagPoseFtc tagPose = detection.ftcPose;

        telemetry.addData("Field Position", fieldPosition);
        telemetry.addData("Field Orientation", fieldOrientation);
        telemetry.addData("Tag Pose", "x %.2f, y %.2f, z %.2f, bearing %.2f, yaw %.2f, pitch %.2f, elevation %.2f, range %.2f, roll %.2f", tagPose.x, tagPose.y, tagPose.z, tagPose.bearing, tagPose.yaw, tagPose.pitch, tagPose.elevation, tagPose.range, tagPose.roll);

        double tagX = fieldPosition.get(1);
        double tagY = fieldPosition.get(0);
        Vector2d tagPoint = new Vector2d(tagX, tagY);

        double poseX = tagPose.x;
        double poseY = tagPose.y;
        double poseYaw = tagPose.yaw;

        Vector2d cameraPoint = offset(tagPoint, poseX, poseYaw);
        cameraPoint = offset(cameraPoint, poseY, poseYaw - 90);

        double cameraX = cameraPoint.getX();
        double cameraY = cameraPoint.getY();
        double cameraHeading = -poseYaw - 90;

        telemetry.addData("camera", "x %.2f, y %.2f, heading %.2f", cameraX, cameraY, cameraHeading);

        Vector2d robotPoint = offset(cameraPoint, 7, -poseYaw - 90);

        double robotX = robotPoint.getX();
        double robotY = robotPoint.getY();
        double robotHeading = cameraHeading + 180;

        telemetry.addData("robot", "x %.2f, y %.2f, heading %.2f", robotX, robotY, robotHeading);

        telemetry.update();

        Pose2d startPose = new Pose2d(robotX, robotY, Math.toRadians(robotHeading));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                //.splineTo(new Vector2d(36, -58), Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(36, -60, Math.toRadians(90)))
                .build();

        drive.followTrajectorySequence(sequence);

        while (opModeIsActive()) {}
    }

    private static Vector2d offset(Vector2d inputPoint, double distance, double degrees) {
        double inputX = inputPoint.getX();
        double inputY = inputPoint.getY();
        double radians = Math.toRadians(degrees);
        double outputX = inputX + distance * Math.cos(radians);
        double outputY = inputY - distance * Math.sin(radians);
        Vector2d outputPoint = new Vector2d(outputX, outputY);
        return outputPoint;
    }

    public void update() throws InterruptedException {
        if (!initialized) {
            throw new InterruptedException("Please call initialize before calling update.");
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private boolean isMatch(AprilTagDetection detection) {
        if (detection.metadata == null) {
            return false;
        }
        boolean isMatch = detection.id == 9;

        return isMatch;
    }

    private AprilTagDetection getDetection() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (isMatch(detection)) {
                return detection;
            }
        }

        return null;
    }

    public void getDistanceFromTag() {
        AprilTagDetection detection = getDetection();

//        detection.metadata.fieldPosition
//        detection.metadata.fieldOrientation
//        detection.ftcPose

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double rangeError = (detection.ftcPose.range);
        double headingError = detection.ftcPose.bearing;
        double yawError = detection.ftcPose.yaw;


    }

    public boolean isActive() {
        return targetMode != HeatSeek.TargetMode.NONE;
    }

    public void initialize() {
        initialized = true;
        // Use low exposure time to reduce motion blur
        setManualExposure(6, 250);
    }
}
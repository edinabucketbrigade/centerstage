package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(preselectTeleOp = "TeleOpA")
//@Disabled
public class AutoG extends LinearOpMode {
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection process.
    private HeatSeek.TargetMode targetMode = HeatSeek.TargetMode.NONE;
    private boolean initialized;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT))
                .addProcessor(aprilTag)
                .build();

        waitForStart();

        AprilTagDetection detection = null;

        while (opModeIsActive() && detection == null) {
            detection = getDetection();

            telemetry.addData("Status", "Running");

            telemetry.update();
        }

        telemetry.addData("Field Position", detection.metadata.fieldPosition);
        telemetry.addData("Field Orientation", detection.metadata.fieldOrientation);
        telemetry.addData("ftcPose", "x %.2f, y %.2f, z %.2f ", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
        telemetry.addData("ftcPose2", "bearing %.2f, yaw %.2f, pitch %.2f, elevation %.2f, range %.2f, roll %.2f", detection.ftcPose.bearing, detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.elevation, detection.ftcPose.range, detection.ftcPose.roll);

        double x = detection.ftcPose.x - detection.metadata.fieldPosition.get(1);
        double y = detection.metadata.fieldPosition.get(0) + detection.ftcPose.y;
        double heading = detection.metadata.fieldOrientation.z - detection.ftcPose.yaw;

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", heading);

        telemetry.update();

        while (opModeIsActive()) {}
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
        boolean isMatch = detection.id == 10 || detection.id == 7;

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
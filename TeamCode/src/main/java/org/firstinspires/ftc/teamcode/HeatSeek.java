package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class HeatSeek {

    enum TargetMode {
        LEFT,
        MIDDLE,
        RIGHT,
        NONE
    }

    enum TargetState {
        POSITIONING,
        LIFTING,
        DROPPING,
        LOWERING
    }

    // This is how close the camera should get to the target when placing high (in inches)
    // - Edina: 6
    // - Prior Lake: 4.25
    public static double HIGH_DESIRED_DISTANCE = 5;
    
    // This is how close the camera should get to the target when placing low (in inches)
    // - Edina: 9.5
    // - Prior Lake: 8.5
    public static double LOW_DESIRED_DISTANCE = 9;

    public static double SPEED_GAIN = 0.04; // Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static double STRAFE_GAIN = 0.015; // Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static double MAX_AUTO_SPEED = 0.5; // Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_STRAFE = 0.5; // Clip the approach speed to this max value (adjust for your robot)

    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection process.
    private TargetMode targetMode = TargetMode.NONE;
    private TargetState targetState = TargetState.POSITIONING;
    private ElapsedTime timer = new ElapsedTime();
    private boolean startedTimer;
    private int framesInPosition;
    private RobotHardwareA robotHardware;
    private boolean initialized;

    public HeatSeek(RobotHardwareA robotHardware) {

        this.robotHardware = robotHardware;

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(robotHardware.opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT))
                .addProcessor(aprilTag)
                .build();

    }

    public void update() throws InterruptedException {
        if (!initialized) {
            throw new InterruptedException("Please call initialize before calling update.");
        }

        if (targetMode == TargetMode.NONE) {
            robotHardware.moveRobot();
        } else {
            if (targetState == TargetState.POSITIONING) {
                AprilTagDetection detection = getDetection();
                if (detection == null) {
                    robotHardware.moveRobot();
                    robotHardware.opMode.telemetry.addData("Note", "Target is missing");
                } else {
                    updateRobot(detection);
                }
            } else {
                updateRobot(null);
            }
        }

    }

    private void initializeTargetState() {
        framesInPosition = 0;
        targetState = TargetState.POSITIONING;
        startedTimer = false;
    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        LinearOpMode opMode = robotHardware.opMode;

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
        }
        if (!opMode.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                opMode.sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            opMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            opMode.sleep(20);
        }
    }

    private boolean isMatch(AprilTagDetection detection) {
        if (detection.metadata == null) {
            return false;
        }
        boolean isMatch =
                (targetMode == TargetMode.LEFT && (detection.id == 1 || detection.id == 4)) ||
                        (targetMode == TargetMode.MIDDLE && (detection.id == 2 || detection.id == 5)) ||
                        (targetMode == TargetMode.RIGHT && (detection.id == 3 || detection.id == 6));

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

    private void handlePositioning(AprilTagDetection detection) {
        robotHardware.raiseWrist();

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double desiredDistance = getDesiredDistance();
        double rangeError = (detection.ftcPose.range - desiredDistance);
        double headingError = detection.ftcPose.bearing;
        double yawError = detection.ftcPose.yaw;

        if (framesInPosition > 10) {
            targetState = TargetState.LIFTING;
            return;
        }

        boolean isInPosition = Math.abs(rangeError) < 1.3 && Math.abs(headingError) < 2;

        if (isInPosition) {
            framesInPosition++;
        } else {
            framesInPosition = 0;
        }

        LinearOpMode opMode = robotHardware.opMode;

        Telemetry telemetry = opMode.telemetry;

        telemetry.addData("Range Error", "%5.1f inches", rangeError);
        telemetry.addData("Heading Error", "%5.1f degrees", headingError);
        telemetry.addData("Yaw Error", "%5.1f degrees", yawError);

        telemetry.addData("Frames In Position", framesInPosition);

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(headingError * RobotHardwareA.TURN_GAIN, -RobotHardwareA.MAX_AUTO_TURN, RobotHardwareA.MAX_AUTO_TURN);
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("Target", "ID %d (%s)", detection.id, detection.metadata.name);
        telemetry.addData("Range", "%5.1f inches", detection.ftcPose.range);
        telemetry.addData("Bearing", "%3.0f degrees", detection.ftcPose.bearing);
        telemetry.addData("Yaw", "%3.0f degrees", detection.ftcPose.yaw);
        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

        robotHardware.moveRobot(drive, strafe, turn);

        opMode.sleep(10);
    }

    private void handleLifting() {
        robotHardware.stopDriveMotors();
        robotHardware.raiseArm();
        if (robotHardware.isArmRaised()) {
            timer.reset();
            targetState = TargetState.DROPPING;
        }
    }

    private void handleDropping() {
        robotHardware.openLeftClaw();
        robotHardware.openRightClaw();
        if (timer.milliseconds() > 1000) {
            targetState = TargetState.LOWERING;
        }
    }

    private void handleLowering() {
        robotHardware.lowerArm();
        robotHardware.lowerWrist();
        targetMode = TargetMode.NONE;
    }

    private void updateRobot(AprilTagDetection detection) throws InterruptedException {
        switch (targetState) {
            case POSITIONING:
                handlePositioning(detection);
                break;
            case LIFTING:
                handleLifting();
                break;
            case DROPPING:
                handleDropping();
                break;
            case LOWERING:
                handleLowering();
                break;
            default:
                throw new InterruptedException("Unrecognized target state.");
        }
        Telemetry telemetry = robotHardware.opMode.telemetry;
        telemetry.addData("Target Mode", targetMode);
        telemetry.addData("Target State", targetState);
        telemetry.addData("Started Timer", startedTimer);
        telemetry.addData("Timer", timer);
    }

    public void startLeft() {
        targetMode = TargetMode.LEFT;
        initializeTargetState();
    }

    public void startRight() {
        targetMode = TargetMode.RIGHT;
        initializeTargetState();
    }

    public void startMiddle() {
        targetMode = TargetMode.MIDDLE;
        initializeTargetState();
    }

    public void cancel() {
        targetMode = TargetMode.NONE;
    }

    private double getDesiredDistance() {
        boolean isHighDrop = robotHardware.isHighDrop;
        return isHighDrop ? HIGH_DESIRED_DISTANCE : LOW_DESIRED_DISTANCE;
    }

    public boolean isActive() {
        return targetMode != TargetMode.NONE;
    }

    public void initialize() {
        initialized = true;
        // Use low exposure time to reduce motion blur
        setManualExposure(6, 250);
    }
}
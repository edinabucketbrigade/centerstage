package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class TeleOpM extends LinearOpMode {

    enum TargetMode {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }
    enum TargetState {
        POSITIONING,
        LIFTING,
        DROPPING,
        LOWERING
    }

    public static double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)
    public static double SPEED_GAIN = 0.03;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static double TURN_GAIN = 0.02;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private RobotHardwareA robotHardware = null;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private TargetMode targetMode = TargetMode.NONE;
    private TargetState targetState = TargetState.POSITIONING;
    private ElapsedTime timer = new ElapsedTime();
    private boolean startedTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean previousB = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousA = false;

        // Initialize the AprilTag Detection process
        initAprilTag();

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        robotHardware = new RobotHardwareA(this);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            robotHardware.update();

            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;
            boolean currentY = gamepad1.y;
            boolean currentA = gamepad1.a;

            if (currentX && !previousX) {
                targetMode = TargetMode.LEFT;
                initializeTargetState();
            }

            if (currentA && !previousA) {
                targetMode = TargetMode.CENTER;
                initializeTargetState();
            }

            if (currentB && !previousB) {
                targetMode = TargetMode.RIGHT;
                initializeTargetState();
            }

            if (currentY && !previousY){
                targetMode = TargetMode.NONE;
            }

            AprilTagDetection detection = null;

            if (targetMode != TargetMode.NONE) {
                detection = getDetection();
                if(detection == null) {
                    telemetry.addData("Note", "Target is missing");
                }
            }

            if(detection == null) {
                robotHardware.moveRobot();
            }
            else {
                updateRobot(detection);
            }

            previousB = currentB;
            previousX = currentX;
            previousY = currentY;
            previousA = currentA;

            telemetry.update();

        }
    }

    private void initializeTargetState() {
        targetState = TargetState.POSITIONING;
        startedTimer = false;
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        robotHardware.moveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT))
                .addProcessor(aprilTag)
                .build();
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
        boolean isMatch =
                (targetMode == TargetMode.LEFT && (detection.id == 1 || detection.id == 4)) ||
                        (targetMode == TargetMode.CENTER && (detection.id == 2 || detection.id == 5)) ||
                        (targetMode == TargetMode.RIGHT && (detection.id == 3 || detection.id == 6));

        return isMatch;
    }

    private AprilTagDetection getDetection() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if(isMatch(detection)) {
                return detection;
            }
        }

        return null;
    }

    private void handlePositioning(AprilTagDetection detection) {
        robotHardware.raiseWrist();

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double rangeError = (detection.ftcPose.range - DESIRED_DISTANCE);
        double headingError = detection.ftcPose.bearing;
        double yawError = detection.ftcPose.yaw;

        if (startedTimer && timer.milliseconds() > 1000) {
            startedTimer = false;
            targetState = TargetState.LIFTING;
            return;
        }

        if (rangeError < 2 && !startedTimer) {
            startedTimer = true;
            timer.reset();
        }

        telemetry.addData("Range Error", "%5.1f inches", rangeError);

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("Target", "ID %d (%s)", detection.id, detection.metadata.name);
        telemetry.addData("Range", "%5.1f inches", detection.ftcPose.range);
        telemetry.addData("Bearing", "%3.0f degrees", detection.ftcPose.bearing);
        telemetry.addData("Yaw", "%3.0f degrees", detection.ftcPose.yaw);
        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        telemetry.addData("Target Mode", targetMode);
        telemetry.addData("Target State", targetState);
        telemetry.addData("Started Timer", startedTimer);
        telemetry.addData("Timer", timer);

        moveRobot(drive, strafe, turn);

        sleep(10);
    }

    private void handleLifting() {
        robotHardware.raiseArm();
        if (robotHardware.isArmRaised()) {
            if (startedTimer && timer.milliseconds() > 500) {
                timer.reset();
                targetState = TargetState.DROPPING;
            }

            if (!startedTimer) {
                startedTimer = true;
                timer.reset();
            }
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
        switch(targetState) {
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
    }
}
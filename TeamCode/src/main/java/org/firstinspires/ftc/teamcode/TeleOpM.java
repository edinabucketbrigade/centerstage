package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class TeleOpM extends LinearOpMode {

    enum TargetMode {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    final double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private RobotHardwareA robotHardware = null;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private TargetMode targetMode = TargetMode.NONE;

    @Override
    public void runOpMode() {

        // Initialize the AprilTag Detection process
        initAprilTag();

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        robotHardware = new RobotHardwareA(this);

        telemetry.update();

        robotHardware.closeLeftClaw();
        robotHardware.closeRightClaw();
        robotHardware.raiseWrist();

        waitForStart();

        while (opModeIsActive()) {

            robotHardware.update();

            if (gamepad1.x) {
                targetMode = TargetMode.LEFT;
            } else if (gamepad1.a) {
                targetMode = TargetMode.CENTER;
            } else if (gamepad1.b) {
                targetMode = TargetMode.RIGHT;
            } else {
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
                driveTowardTarget(detection);
            }

            telemetry.update();

        }
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

    private void driveTowardTarget(AprilTagDetection detection) {
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double rangeError = (detection.ftcPose.range - DESIRED_DISTANCE);
        double headingError = detection.ftcPose.bearing;
        double yawError = detection.ftcPose.yaw;

        if (rangeError < 2) {
            robotHardware.raiseArm();
        }
        if (robotHardware.isArmRaised()) {
            robotHardware.openLeftClaw();
            robotHardware.openRightClaw();
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

        moveRobot(drive, strafe, turn);

        sleep(10);
    }
}
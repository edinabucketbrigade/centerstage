    package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.tensorflow.lite.task.vision.detector.Detection;

import java.util.List;
import java.util.concurrent.TimeUnit;


enum TargetMode {
    LEFT,
    CENTER,
    RIGHT,
    NONE
}

    @TeleOp
    public class TeleOpM extends LinearOpMode {

        final double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)
        final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)TeleOpC
        final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

        private RobotHardwareA robotHardware = null;

        private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
        private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
        private VisionPortal visionPortal;               // Used to manage the video source.
        private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
        private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
        private TargetMode targetMode = TargetMode.NONE;

        @Override
        public void runOpMode() {
            boolean targetFound = false;    // Set to true when an AprilTag target is detected
            double drive = 0;        // Desired forward power/speed (-1 to +1)
            double strafe = 0;        // Desired strafe power/speed (-1 to +1)
            double turn = 0;        // Desired turning power/speed (-1 to +1)

            // Initialize the Apriltag Detection process
            initAprilTag();

            robotHardware = new RobotHardwareA(this);

            if (USE_WEBCAM)
                setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

            // Wait for driver to press start
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {

                if (gamepad1.x) {
                    targetMode = TargetMode.LEFT;
                } else if (gamepad1.a) {
                    targetMode = TargetMode.CENTER;
                } else if (gamepad1.b) {
                    targetMode = TargetMode.RIGHT;
                } else {
                    targetMode = TargetMode.NONE;
                }

                telemetry.addData("TargetMode", targetMode);

                if (targetMode != TargetMode.NONE) {

                    telemetry.addData("Debug", "targetmode is not none");

                    targetFound = false;
                    desiredTag = null;

                    // Step through the list of detected tags and look for a matching tag
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        targetFound = isMatch(detection);

                        if (targetFound) {
                            desiredTag = detection;
                            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                            break;  // don't look any further.
                        } else {
                            telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                        }
                    }
                    if (!targetFound) {
                        telemetry.addData(">", "Drive using joysticks to find valid target\n");
                    }

                    if (targetFound) {

                        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double headingError = desiredTag.ftcPose.bearing;
                        double yawError = desiredTag.ftcPose.yaw;

                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                    } else {
                        drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                        strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
                        turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                        telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                    }

                    moveRobot(drive, strafe, turn);
                    sleep(10);
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
            robotHardware.setDrivePowerManually(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }

        private void initAprilTag() {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder().build();

            // Create the vision portal by using a builder.
            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }

        private void setManualExposure(int exposureMS, int gain) {
            // Wait for the camera to be open, then use the controls

            if (visionPortal == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
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
    }

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

enum Route {
    RED_LEFT_DIRECT,
    RED_LEFT_INDIRECT,
    RED_RIGHT_DIRECT,
    RED_RIGHT_INDIRECT,
    BLUE_LEFT_DIRECT,
    BLUE_LEFT_INDIRECT,
    BLUE_RIGHT_DIRECT,
    BLUE_RIGHT_INDIRECT,
    SPEED_TEST,
    CYCLE_TEST,
    PARK_LEFT,
    PARK_RIGHT

}

@Autonomous(preselectTeleOp = "TeleOpA")
//@Disabled
public class AutoF extends LinearOpMode {
    public static final Vector2d RED_MIDDLE = new Vector2d(0,-8);
    public  static final Vector2d RED_DETOUR_BACKDROP = new Vector2d(28,-8);
    public static final Vector2d RED_BACKDROP = new Vector2d(44,-36);
    public static final Vector2d RED_PIXELS = new Vector2d(-58,-10);
    public static final Vector2d RED_LEFT_START = new Vector2d(-36,-61);
    public static final Vector2d RED_RIGHT_START = new Vector2d(12,-61);
    public static final Vector2d RED_RIGHT_LEFT_POSITION = new Vector2d(14,-30);
    public static final Vector2d RED_RIGHT_RIGHT_POSITION = new Vector2d(23,-30);
    public static final Vector2d RED_LEFT_RIGHT_RIGGING_POSITION = new Vector2d(-9.5, -35);
    public static final Vector2d RED_LEFT_LEFT_SPIKE_MARK_POSITION = new Vector2d(-34,-34);

    public static final Vector2d BLUE_MIDDLE = new Vector2d(0,8);
    public  static final Vector2d BLUE_DETOUR_BACKDROP = new Vector2d(28,8);
    public static final Vector2d BLUE_BACKDROP = new Vector2d(44,36);
    public static final Vector2d BLUE_PIXELS = new Vector2d(-58,10);
    public static final Vector2d BLUE_LEFT_START = new Vector2d(12,61);
    public static final Vector2d BLUE_RIGHT_START = new Vector2d(-36,61);
    public static final Vector2d BLUE_RIGHT_LEFT_POSITION = new Vector2d(23,30);
    public static final Vector2d BLUE_RIGHT_RIGHT_POSITION = new Vector2d(14,30);
    public static final Vector2d BLUE_LEFT_LEFT_RIGGING_POSITION = new Vector2d(-9.5,35);
    public static final Vector2d BLUE_LEFT_RIGHT_SPIKE_MARK_POSITION = new Vector2d(-34,34);

    private Boolean redAlliance = null;
    private Boolean startLeft = null;
    private Boolean parkLeft = null;

    OpenCvWebcam camera;
    private boolean startedStreaming;



    public static final Route ROUTE = Route.RED_LEFT_DIRECT;
    public static final double DELAY = 0.5;
    @Override
    public void runOpMode() {
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        while (true) {
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        CenterStageCVDetection detector = new CenterStageCVDetection(parkLeft, redAlliance, startLeft, telemetry);
        camera.setPipeline(detector);
        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        log("opening camera");
        /*
         *   Below is an example of a lambda expression which is in simply an anonymous function.
         *   Since we are only executing one statement we are able to remove the curly braces and semicolon
         *   making it look much cleaner.
         *   Note that this is a feature strictly for SDK 8+, if Java 7 is being used use this code instead.
         *   To change preferences press command and ; to open up preference window.
         *
         *   * Lambda Expression *
         *   camera.openCameraDeviceAsync(() -> camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT));
         */
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                camera.startStreaming(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                startedStreaming = true;
                log("started camera streaming");
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
                log("error opening camera: " + errorCode);
            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        while (opModeIsActive() && !startedStreaming) {
            log("waiting for camera streaming to start");
            sleep(50);
        }

        CenterStageCVDetection.Location location = null;

        while (opModeIsActive() && location == null) {
            log("waiting for location detection");
            sleep(50);
            location = detector.getLocation();
        }

        waitForStart();

        location = detector.getLocation();

        camera.stopStreaming();

        camera.closeCameraDevice();

        if (isStopRequested()) return;

        TrajectorySequence redLeftLeftSequence = getRedLeftLeftTrajectorySequence(drive);
        TrajectorySequence redLeftMiddleSequence = getRedLeftMiddleTrajectorySequence(drive);
        TrajectorySequence redLeftRightSequence = getRedLeftMiddleTrajectorySequence(drive);

        TrajectorySequence redRightLeftSequence = getRedLeftLeftTrajectorySequence(drive);
        TrajectorySequence redRightMiddleSequence = getRedLeftMiddleTrajectorySequence(drive);
        TrajectorySequence redRightRightSequence = getRedLeftMiddleTrajectorySequence(drive);

        TrajectorySequence blueLeftLeftSequence = getRedLeftLeftTrajectorySequence(drive);
        TrajectorySequence blueLeftMiddleSequence = getRedLeftMiddleTrajectorySequence(drive);
        TrajectorySequence blueLeftRightSequence = getRedLeftMiddleTrajectorySequence(drive);

        TrajectorySequence blueRightLeftSequence = getRedLeftLeftTrajectorySequence(drive);
        TrajectorySequence blueRightMiddleSequence = getRedLeftMiddleTrajectorySequence(drive);
        TrajectorySequence blueRightRightSequence = getRedLeftMiddleTrajectorySequence(drive);


        if (redAlliance) {
            if (startLeft) {
                if (location == location.Left) {
                    drive.followTrajectorySequence(redLeftLeftSequence);
                }
                if (location == location.Middle) {
                    drive.followTrajectorySequence(redLeftMiddleSequence);
                }
                if (location == location.Right) {
                    drive.followTrajectorySequence(redLeftRightSequence);
                }
            }
            else {
                if (location == location.Left) {
                    drive.followTrajectorySequence(redRightLeftSequence);
                }
                if (location == location.Middle) {
                    drive.followTrajectorySequence(redRightMiddleSequence);
                }
                if (location == location.Right) {
                    drive.followTrajectorySequence(redRightRightSequence);
                }
            }
        }
        else {
            if (startLeft) {
                if (location == location.Left) {
                    drive.followTrajectorySequence(blueLeftLeftSequence);
                }
                if (location == location.Middle) {
                    drive.followTrajectorySequence(blueLeftMiddleSequence);
                }
                if (location == location.Right) {
                    drive.followTrajectorySequence(blueLeftRightSequence);
                }
            }
            else {
                if (location == location.Left) {
                    drive.followTrajectorySequence(blueRightLeftSequence);
                }
                if (location == location.Middle) {
                    drive.followTrajectorySequence(blueRightMiddleSequence);
                }
                if (location == location.Right) {
                    drive.followTrajectorySequence(blueRightRightSequence);
                }
            }
        }
    }
// Red

    // Left

        // Left
        private TrajectorySequence getRedLeftLeftTrajectorySequence(SampleMecanumDrive drive) {
            Pose2d startPose = new Pose2d(RED_LEFT_START, Math.toRadians(-90));
            drive.setPoseEstimate(startPose);

            TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(RED_LEFT_LEFT_SPIKE_MARK_POSITION,Math.toRadians(0)))
                    .strafeLeft(20)
                    .turn(Math.toRadians(-90))
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
                    .splineTo(RED_LEFT_RIGHT_RIGGING_POSITION,Math.toRadians(0))
                    .turn(Math.toRadians(90))
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

        // Left
    private TrajectorySequence getRedRightLeftTrajectorySequence(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(RED_RIGHT_LEFT_POSITION, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(RED_BACKDROP, Math.toRadians(180)))
                .setReversed(false)
                .splineTo(RED_MIDDLE,Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP,Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_MIDDLE,Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP,Math.toRadians(0))
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
                .splineTo(RED_MIDDLE,Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP,Math.toRadians(0))
                .setReversed(false)
                .splineTo(RED_MIDDLE,Math.toRadians(180))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(RED_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(RED_MIDDLE,Math.toRadians(0))
                .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(RED_BACKDROP,Math.toRadians(0))
                .build();
        return sequence;
    }

        // Right
        private TrajectorySequence getRedRightRightrajectorySequence(SampleMecanumDrive drive) {
            Pose2d startPose = new Pose2d(RED_RIGHT_START, Math.toRadians(-90));
            drive.setPoseEstimate(startPose);

            TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineTo((RED_RIGHT_RIGHT_POSITION), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(RED_BACKDROP, Math.toRadians(180)))
                    .setReversed(false)
                    .splineTo(RED_MIDDLE,Math.toRadians(180))
                    .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                    .splineTo(RED_PIXELS, Math.toRadians(180))
                    .setReversed(true)
                    .splineTo(RED_MIDDLE,Math.toRadians(0))
                    .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(RED_BACKDROP,Math.toRadians(0))
                    .setReversed(false)
                    .splineTo(RED_MIDDLE,Math.toRadians(180))
                    .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(180))
                    .splineTo(RED_PIXELS, Math.toRadians(180))
                    .setReversed(true)
                    .splineTo(RED_MIDDLE,Math.toRadians(0))
                    .splineTo(RED_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(RED_BACKDROP,Math.toRadians(0))
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
                    .splineTo(BLUE_LEFT_LEFT_RIGGING_POSITION,Math.toRadians(0))
                    .turn(Math.toRadians(90))
                    .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                    .setReversed(false)
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                    .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                    .splineTo(BLUE_PIXELS, Math.toRadians(180))
                    .setReversed(true)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                    .setReversed(false)
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                    .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                    .splineTo(BLUE_PIXELS, Math.toRadians(180))
                    .setReversed(true)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(BLUE_BACKDROP,Math.toRadians(0))
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
                .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                .build();
        return sequence;
    }

            // Right
        private TrajectorySequence getBlueLeftRightTrajectorySequence(SampleMecanumDrive drive) {
            Pose2d startPose = new Pose2d(BLUE_LEFT_START, Math.toRadians(90));
            drive.setPoseEstimate(startPose);

            TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(BLUE_LEFT_RIGHT_SPIKE_MARK_POSITION,Math.toRadians(0)))
                    .strafeLeft(20)
                    .turn(Math.toRadians(-90))
                    .setReversed(true)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(BLUE_BACKDROP, Math.toRadians(0))
                    .setReversed(false)
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                    .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                    .splineTo(BLUE_PIXELS, Math.toRadians(180))
                    .setReversed(true)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                    .setReversed(false)
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                    .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                    .splineTo(BLUE_PIXELS, Math.toRadians(180))
                    .setReversed(true)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                    .build();
            return sequence;
        }

    // Right

        // Right
        private TrajectorySequence getBlueRightLefttrajectorySequence(SampleMecanumDrive drive) {
            Pose2d startPose = new Pose2d(BLUE_RIGHT_START, Math.toRadians(90));
            drive.setPoseEstimate(startPose);

            TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineTo((BLUE_RIGHT_LEFT_POSITION), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(BLUE_BACKDROP, Math.toRadians(180)))
                    .setReversed(false)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                    .splineTo(BLUE_PIXELS, Math.toRadians(180))
                    .setReversed(true)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                    .setReversed(false)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                    .splineTo(BLUE_PIXELS, Math.toRadians(180))
                    .setReversed(true)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(BLUE_BACKDROP,Math.toRadians(0))
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
                .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                .setReversed(false)
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                .splineTo(BLUE_PIXELS, Math.toRadians(180))
                .setReversed(true)
                .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                .splineTo(BLUE_BACKDROP,Math.toRadians(0))
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
                    .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                    .splineTo(BLUE_PIXELS, Math.toRadians(180))
                    .setReversed(true)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                    .setReversed(false)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(180))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(180))
                    .splineTo(BLUE_PIXELS, Math.toRadians(180))
                    .setReversed(true)
                    .splineTo(BLUE_MIDDLE,Math.toRadians(0))
                    .splineTo(BLUE_DETOUR_BACKDROP, Math.toRadians(0))
                    .splineTo(BLUE_BACKDROP,Math.toRadians(0))
                    .build();
            return sequence;
        }

    private void log(String message) {
        telemetry.addData("Message", message);
        telemetry.update();
    }

}
package org.firstinspires.ftc.teamcode;

import static bucketbrigade.casperlibrary.TeamPropLocation.LEFT;
import static bucketbrigade.casperlibrary.TeamPropLocation.MIDDLE;
import static bucketbrigade.casperlibrary.TeamPropLocation.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import bucketbrigade.casperlibrary.Objectives;
import bucketbrigade.casperlibrary.TeamPropLocation;

@Config
@Autonomous(preselectTeleOp = "TeleOpM")
@Disabled
public class CenterStageAutoCV extends LinearOpMode {
    /*
     * If program has a build folder error try clearing the build
     */
    public static int AWAY_FORWARD = 820;
    public static int TOWARD_FORWARD = 1310;
    public static int MIDDLE_FORWARD_POSITION = 1245;
    public static int LEFT_TOWARD_STRAFE = -520;
    public static int RIGHT_TOWARD_STRAFE = -600;
    public static double WHEEL_POWER = 0.5;
    public static int RIGHT_OFFSET_STRAFE = 640;
    public static int LEFT_OFFSET_STRAFE = 520;
    public static int MIDDLE_OFFSET_STRAFE = 200;
    public static int FAR_DISTANCE_TO_BACKDROP = 3000;
    public static int CLOSE_DISTANCE_TO_BACKDROP = 850;
    public static int MIDDLE_BACK_UP = 1200;
    public static int BACKUP = 650;
    public static int SMALL_BACKUP = 200;
    public static int SORTA_EXTRA_BACKUP = 750;
    public static int EXTRA_BACKUP = 1200;
    public static int EXTRA_EXTRA_BACKUP = 1400;
    public static int SAME_DIRECTION_PARK_DISTANCE = 1000;
    public static int DIFFERENT_DIRECTION_PARK_DISTANCE = 1700;
    public static int MIDDLE_PARK_DISTANCE = 1400;
    private Boolean redAlliance = null;
    private Boolean startLeft = null;
    private Boolean parkLeft = null;
    OpenCvWebcam camera;

    private RobotHardwareA robotHardware = null;
    private boolean startedStreaming;

    @Override
    public void runOpMode() throws InterruptedException {
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

        robotHardware = new RobotHardwareA(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        CenterStageCVDetection detector = new CenterStageCVDetection(telemetry, false, new LaunchMenu(this));
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
        robotHardware.closeLeftClaw();
        robotHardware.closeRightClaw();
        sleep(2000);
        robotHardware.raiseWrist();

        while (opModeIsActive() && !startedStreaming) {
            log("waiting for camera streaming to start");
            sleep(50);
        }

        TeamPropLocation location = null;

        while (opModeIsActive() && location == null) {
            log("waiting for location detection");
            sleep(50);
            location = detector.getLocation();
        }

        waitForStart();

        location = detector.getLocation();

        camera.stopStreaming();

        camera.closeCameraDevice();

        robotHardware.resetYaw();

        placePixelOnSpikeMark(location);
        placePixelOnBackdrop(location);
        park(location);

        while (opModeIsActive()) {
        }
    }

    private void placePixelOnSpikeMark(TeamPropLocation location) {
        switch (location) {
            case RIGHT:
                placePixelRight();
                break;
            case LEFT:
                placePixelLeft();
                break;
            case MIDDLE:
                placePixelMiddle();
                break;
        }
        robotHardware.openRightClaw();
    }

    private void placePixelOnBackdrop(TeamPropLocation location) throws InterruptedException {
        HeatSeekA heatSeek = new HeatSeekA(robotHardware);

        if (redAlliance) {
            if (startLeft) {
                if (location == LEFT) {
                    robotHardware.raiseWrist();
                    moveForward(BACKUP);
                    robotHardware.turnToHeading(-90);
                    moveForward(FAR_DISTANCE_TO_BACKDROP);
                    moveRight(-EXTRA_BACKUP);
                }
                if (location == MIDDLE) {
                    moveForward(MIDDLE_BACK_UP);
                    robotHardware.turnToHeading(-90);
                    moveForward(FAR_DISTANCE_TO_BACKDROP);
                    moveRight(-EXTRA_BACKUP);
                }
                if (location == RIGHT) {
                    moveForward(SMALL_BACKUP);
                    robotHardware.raiseWrist();
                    moveRight(-EXTRA_EXTRA_BACKUP);
                    robotHardware.turnToHeading(-90);
                    moveForward(FAR_DISTANCE_TO_BACKDROP);
                    moveRight(-BACKUP);
                }
            } else {
                if (location == LEFT) {
                    moveForward(CLOSE_DISTANCE_TO_BACKDROP);
                }
                if (location == MIDDLE) {
                    moveForward(SMALL_BACKUP);
                    robotHardware.turnToHeading(-90);
                    moveForward(CLOSE_DISTANCE_TO_BACKDROP);
                    moveRight(-SMALL_BACKUP);
                }
                if (location == RIGHT) {
                    robotHardware.raiseWrist();
                    moveForward(SMALL_BACKUP);
                    robotHardware.turnToHeading(-90);
                    moveForward(CLOSE_DISTANCE_TO_BACKDROP);
                    moveRight(-SMALL_BACKUP);
                }
            }
        } else {
            if (startLeft) {
                if (location == LEFT) {
                    robotHardware.raiseWrist();
                    moveForward(SMALL_BACKUP);
                    robotHardware.turnToHeading(90);
                    moveForward(CLOSE_DISTANCE_TO_BACKDROP);
                    moveRight(SMALL_BACKUP);
                }
                if (location == MIDDLE) {
                    moveForward(SMALL_BACKUP);
                    robotHardware.turnToHeading(90);
                    moveForward(CLOSE_DISTANCE_TO_BACKDROP);
                    moveRight(SMALL_BACKUP);
                }
                if (location == RIGHT) {
                    moveForward(CLOSE_DISTANCE_TO_BACKDROP);
                }
            } else {
                if (location == LEFT) {
                    moveForward(SMALL_BACKUP);
                    robotHardware.raiseWrist();
                    sleep(1000);
                    moveRight(EXTRA_EXTRA_BACKUP);
                    robotHardware.turnToHeading(90);
                    moveForward(FAR_DISTANCE_TO_BACKDROP);
                    moveRight(BACKUP);
                }
                if (location == MIDDLE) {
                    moveForward(MIDDLE_BACK_UP);
                    robotHardware.turnToHeading(90);
                    moveForward(FAR_DISTANCE_TO_BACKDROP);
                    moveRight(EXTRA_BACKUP);
                }
                if (location == RIGHT) {
                    robotHardware.raiseWrist();
                    moveForward(SORTA_EXTRA_BACKUP);
                    robotHardware.turnToHeading(90);
                    moveForward(FAR_DISTANCE_TO_BACKDROP);
                    moveRight(EXTRA_BACKUP);
                }
            }
            robotHardware.turnToHeading(90);
        }

        robotHardware.runUsingEncoder(0);

        heatSeek.initialize();

        if (location == LEFT) {
            heatSeek.startLeft();
        } else if (location == MIDDLE) {
            heatSeek.startMiddle();
        } else if (location == RIGHT) {
            heatSeek.startRight();
        } else {
            throw new InterruptedException("Unrecognized location.");
        }

        while (opModeIsActive()) {
            if (!heatSeek.isActive()) {
                break;
            }
            robotHardware.update();
            heatSeek.update();
            telemetry.update();
        }
    }

    private void park(TeamPropLocation location) {
        if (location == LEFT && parkLeft) {
            moveRight(-SAME_DIRECTION_PARK_DISTANCE);
        }
        else if (location == RIGHT && !parkLeft) {
            moveRight(SAME_DIRECTION_PARK_DISTANCE);
        }
        else if (location == LEFT && !parkLeft) {
            moveRight(DIFFERENT_DIRECTION_PARK_DISTANCE);
        }
        else if (location == RIGHT && parkLeft) {
            moveRight(-DIFFERENT_DIRECTION_PARK_DISTANCE);
        }
        else if (location == MIDDLE && !parkLeft) {
            moveRight(MIDDLE_PARK_DISTANCE);
        }
        else if (location == MIDDLE && parkLeft) {
            moveRight(-MIDDLE_PARK_DISTANCE);
        }
    }

    private void placePixelMiddle() {
        moveForward(MIDDLE_FORWARD_POSITION / 2);
        int offsetStrafe = startLeft ? -MIDDLE_OFFSET_STRAFE : MIDDLE_OFFSET_STRAFE;
        moveRight(offsetStrafe);
        robotHardware.lowerWrist();
        robotHardware.turnToHeading(-180);
        moveRight(offsetStrafe);
        moveForward(-MIDDLE_FORWARD_POSITION / 2);
    }

    private void placePixelLeft() {
        int offsetStrafe = startLeft ? -LEFT_OFFSET_STRAFE : LEFT_OFFSET_STRAFE;
        if(startLeft) {
            moveForward(AWAY_FORWARD / 2);
            moveRight(offsetStrafe);
            robotHardware.lowerWrist();
            robotHardware.turnToHeading(180);
            moveForward(-AWAY_FORWARD / 2);
        }
        else {
            moveForward(TOWARD_FORWARD);
            moveRight(offsetStrafe);
            robotHardware.lowerWrist();
            robotHardware.turnToHeading(-90);
            moveForward(LEFT_TOWARD_STRAFE);
        }
    }

    private void placePixelRight() {
        int offsetStrafe = startLeft ? -RIGHT_OFFSET_STRAFE : RIGHT_OFFSET_STRAFE;
        if(startLeft) {
            moveForward(TOWARD_FORWARD);
            moveRight(offsetStrafe);
            robotHardware.lowerWrist();
            robotHardware.turnToHeading(90);
            moveForward(RIGHT_TOWARD_STRAFE);
        }
        else {
            moveForward(AWAY_FORWARD / 2);
            moveRight(offsetStrafe);
            robotHardware.lowerWrist();
            robotHardware.turnToHeading(180);
            moveForward(-AWAY_FORWARD / 2);
        }
    }

    private void log(String message) {
        robotHardware.log(message);
        telemetry.addData("Message", message);
        telemetry.update();
    }

    private void moveForward(int position) {
        robotHardware.runToPosition(position, position, position, position, WHEEL_POWER);
    }
    private void moveRight(int position) {
        robotHardware.runToPosition(-position, position, position, -position, WHEEL_POWER);
    }
}

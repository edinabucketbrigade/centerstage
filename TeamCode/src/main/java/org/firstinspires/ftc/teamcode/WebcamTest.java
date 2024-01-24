package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@TeleOp
public class WebcamTest extends LinearOpMode {

    // Streaming value
    boolean isStreaming;

    // Runs the op mode.
    @Override
    public void runOpMode() {

        // Initialize the camera.
        log("initializing camera");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.

        // Open the camera.
        log("opening camera");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                isStreaming = true;
                log("opened camera");
            }

            @Override
            public void onError(int errorCode) {
                log("error opening camera: " + errorCode);
            }
        });

        // Wait for streaming to start.
        while (!isStopRequested() && !isStreaming) {
            log("waiting for camera to start streaming");
            sleep(50);
        }

        // Show the camera stream in the FTC dashboard.
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        // Indicate that the camera is streaming.
        log("camera is streaming");

        // Wait for the user to press start.
        waitForStart();

        // Loop while the op mode is active.
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }

    private void log(String message) {
        telemetry.addData("Message", message);
        telemetry.update();
    }

}
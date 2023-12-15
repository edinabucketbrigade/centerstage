package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class CameraPixelAutoDetection extends LinearOpMode {
    OpenCvWebcam camera;
    boolean startedStreaming = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        CameraPixelColor detector = new CameraPixelColor(telemetry);
        camera.setPipeline(detector);
        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        if (currentGamepad.x && !previousGamepad.x){
            Core.inRange(detector.mat,detector.MINIMUM_WHITE,detector.MAXIMUM_WHITE,detector.mat);
        }
        if (currentGamepad.a && !previousGamepad.a){
            Core.inRange(detector.mat,detector.MINIMUM_WHITE,detector.MAXIMUM_WHITE,detector.mat);
        }
        if (currentGamepad.b && !previousGamepad.b){
            Core.inRange(detector.mat,detector.MINIMUM_WHITE,detector.MAXIMUM_WHITE,detector.mat);
        }
        if (currentGamepad.y && !previousGamepad.y){
            Core.inRange(detector.mat,detector.MINIMUM_WHITE,detector.MAXIMUM_WHITE,detector.mat);
        }
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                startedStreaming = true;
                log("started camera streaming");
            }

            @Override
            public void onError(int errorCode) {
                log("Error"+errorCode);
            }
        });
        log("1");
        while (opModeIsActive() && !startedStreaming) {
            log("waiting for camera streaming to start");
            sleep(50);
        }
        CameraPixelColor.Location location = null;
        log("2");
        while (opModeIsActive() && location == null) {
            log("waiting for location detection");
            sleep(50);
            location = detector.getLocation();
        }
        log("3");
        waitForStart();
        log("4");

        location = detector.getLocation();
        log("5");

        while(opModeIsActive()) {
            detector.telemetry.update();
            telemetry.update();
        }

        camera.stopStreaming();

        camera.closeCameraDevice();
    }
    private void log(String message) {
        telemetry.addData("Message", message);
        telemetry.update();
    }
}

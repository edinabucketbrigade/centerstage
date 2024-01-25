package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
@Disabled
public class CameraPixelAutoDetection extends LinearOpMode {
    OpenCvWebcam camera;
    boolean startedStreaming = false;
    RevBlinkinLedDriver blinkinLedDriver1;
    RevBlinkinLedDriver blinkinLedDriver2;
    RevBlinkinLedDriver.BlinkinPattern blinkinPattern1;
    RevBlinkinLedDriver.BlinkinPattern blinkinPattern2;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        blinkinLedDriver1 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin1");
        blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver1.setPattern(blinkinPattern1);
//        blinkinLedDriver2 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin2");
//        blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
//        blinkinLedDriver2.setPattern(blinkinPattern2);
        CameraPixelColor detector = new CameraPixelColor(telemetry);
        camera.setPipeline(detector);
        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
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
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        while (opModeIsActive() && !startedStreaming) {
            log("waiting for camera streaming to start");
            sleep(50);
        }
        waitForStart();

        while(opModeIsActive()) {
            CameraPixelColor.Pixel leftPixel = detector.getLeftPixel();
            if (leftPixel == CameraPixelColor.Pixel.White)
                blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            if (leftPixel == CameraPixelColor.Pixel.Yellow)
                blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            if (leftPixel == CameraPixelColor.Pixel.Green)
                blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            if (leftPixel == CameraPixelColor.Pixel.Purple)
                blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
            if (leftPixel == CameraPixelColor.Pixel.None)
                blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            blinkinLedDriver1.setPattern(blinkinPattern1);
        }

        camera.stopStreaming();

        camera.closeCameraDevice();
    }
    private void log(String message) {
        telemetry.addData("Message", message);
        telemetry.update();
    }
}

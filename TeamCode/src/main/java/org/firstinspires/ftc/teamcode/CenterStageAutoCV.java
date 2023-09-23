package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.jar.Attributes;

@Autonomous
public class CenterStageAutoCV extends LinearOpMode {
    OpenCvWebcam camera;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        CenterStageCVDetection detector = new CenterStageCVDetection(telemetry);
        camera.setPipeline(detector);
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
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();
        switch (detector.getLocation()) {
            case Left:
                // ...
                break;
            case Right:
                // ...
                break;
            case Middle:
                // ...
                break;
        }
        camera.stopStreaming();
    }
}

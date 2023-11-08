package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class CenterStageAutoCV extends LinearOpMode {
    /*
    * If program has a build folder error try clearing the build
    */
    public static int STRAFE_FORWARD_POSITION = 1000;
    public static int MIDDLE_FORWARD_POSITION = 2150;
    public static int LEFT_POSITION = 450;
    public static int RIGHT_POSITION = 750;
    public static int ROTATE_POSITION = 1400;
    public static double WHEEL_POWER = 0.5;
    OpenCvWebcam camera;

    private RobotHardwareA robotHardware = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardwareA(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        CenterStageCVDetection detector = new CenterStageCVDetection(telemetry);
        camera.setPipeline(detector);
        camera.setMillisecondsPermissionTimeout(1000); // Timeout for obtaining permission is configurable. Set before opening.
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
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
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
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);
        robotHardware.closeLeftClaw();
        robotHardware.closeRightClaw();
        robotHardware.raiseWrist();

        waitForStart();

        CenterStageCVDetection.Location location = detector.getLocation();

        camera.stopStreaming();

        if(location == null) {
            telemetry.addData("Note", "Location is missing.  Give it more time.");
            telemetry.update();
            sleep(5000);
            return;
        }

        robotHardware.resetYaw();

        switch (location) {
            case Right:
                robotHardware.runToPosition(STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION, WHEEL_POWER);
                robotHardware.runToPosition(ROTATE_POSITION,ROTATE_POSITION,-ROTATE_POSITION,-ROTATE_POSITION,WHEEL_POWER);
                robotHardware.lowerWrist();
                robotHardware.runToPosition(RIGHT_POSITION, RIGHT_POSITION, RIGHT_POSITION, RIGHT_POSITION, WHEEL_POWER);
                break;
            case Left:
                robotHardware.runToPosition(STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION, WHEEL_POWER);
                robotHardware.runToPosition(ROTATE_POSITION,ROTATE_POSITION,-ROTATE_POSITION,-ROTATE_POSITION,WHEEL_POWER);
                robotHardware.lowerWrist();
                robotHardware.runToPosition(LEFT_POSITION, LEFT_POSITION, LEFT_POSITION, LEFT_POSITION, WHEEL_POWER);
                break;
            case Middle:
                robotHardware.lowerWrist();
                robotHardware.runToPosition(MIDDLE_FORWARD_POSITION, MIDDLE_FORWARD_POSITION, MIDDLE_FORWARD_POSITION, MIDDLE_FORWARD_POSITION, WHEEL_POWER);
                robotHardware.turnToHeading(90);
                break;
        }
        robotHardware.openLeftClaw();
        robotHardware.openRightClaw();
        while(opModeIsActive()){}
    }
}

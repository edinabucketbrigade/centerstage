package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.jar.Attributes;

@Autonomous
public class CenterStageAutoCV extends LinearOpMode {
    /*
    * If program has a build folder error try clearing the build
    */
    private DcMotor frontRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive =null;
    public static int LEFT = 0;
    public static int RIGHT = 0;
    public static int MIDDLE = 500;

    OpenCvWebcam camera;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware variables
            frontRightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            frontLeftDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            backRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
            backLeftDrive = hardwareMap.get(DcMotor.class, "left_back_ drive");

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int position = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        CenterStageCVDetection detector = new CenterStageCVDetection(telemetry);
        camera.setPipeline(detector);
        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
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
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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

        waitForStart();
        switch (detector.getLocation()) {
            case Left:
                frontRightDrive.setTargetPosition(MIDDLE);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightDrive.setPower(0.1);
                frontLeftDrive.setTargetPosition(LEFT);
                frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeftDrive.setPower(0.1);
                backRightDrive.setTargetPosition(LEFT);
                backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightDrive.setPower(0.1);
                backLeftDrive.setTargetPosition(LEFT);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftDrive.setPower(0.1);
                break;
            case Right:
                frontRightDrive.setTargetPosition(RIGHT);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightDrive.setPower(0.1);
                frontLeftDrive.setTargetPosition(RIGHT);
                frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeftDrive.setPower(0.1);
                backRightDrive.setTargetPosition(RIGHT);
                backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightDrive.setPower(0.1);
                backLeftDrive.setTargetPosition(RIGHT);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftDrive.setPower(0.1);
                break;
            case Middle:
                frontRightDrive.setTargetPosition(MIDDLE);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightDrive.setPower(0.1);
                frontLeftDrive.setTargetPosition(MIDDLE);
                frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeftDrive.setPower(0.1);
                backRightDrive.setTargetPosition(MIDDLE);
                backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightDrive.setPower(0.1);
                backLeftDrive.setTargetPosition(MIDDLE);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftDrive.setPower(0.1);
                break;
        }
        camera.stopStreaming();
        frontLeftDrive.setPower(0);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setPower(0);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setPower(0);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setPower(0);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

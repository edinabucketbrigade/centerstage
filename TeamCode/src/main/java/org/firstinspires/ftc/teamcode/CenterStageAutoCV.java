package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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

@Autonomous
public class CenterStageAutoCV extends LinearOpMode {
    /*
    * If program has a build folder error try clearing the build
    */
    private DcMotor frontRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive =null;
    private DcMotorEx armMotor;
    public static int STRAFE_FORWARD_POSITION = 1800;
    public static int MIDDLE_FORWARD_POSITION = 2150;
    public static int LEFT_POSITION = 450;
    public static int RIGHT_POSITION = 750;
    public static double WHEEL_POWER = 0.5;
    public static int targetArmPosition = 0;
    public static double LEFT_CLAW_MINIMUM = 0.275;
    public static double LEFT_CLAW_MAXIMUM = 0.6;
    public static double RIGHT_CLAW_MINIMUM = 0.7;
    public static double RIGHT_CLAW_MAXIMUM = 1;
    public static double WRIST_MINIMUM = 0;
    public static double WRIST_MAXIMUM = 0.72;
    public static double armPower = 0;
    public boolean armIsReady;
    private TouchSensor touchSensor;
    public static int ARM_MINIMUM = 0;
    public static int ARM_MAXIMUM = 1300;
    public static double ARM_RAISE_POWER = 0.8;
    public static double ARM_LOWER_POWER = 0.4;

    // Declare motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo wristServo = null;
    private Servo leftClawServo = null;
    private Servo rightClawServo = null;



    OpenCvWebcam camera;

    public void runToPosition(int frontRightPosition, int frontLeftPosition, int backRightPosition, int backLeftPosition){
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightDrive.setTargetPosition(frontRightPosition);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setPower(WHEEL_POWER);
        frontLeftDrive.setTargetPosition(frontLeftPosition);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setPower(WHEEL_POWER);
        backRightDrive.setTargetPosition(backRightPosition);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setPower(WHEEL_POWER);
        backLeftDrive.setTargetPosition(backLeftPosition);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setPower(WHEEL_POWER);
        while(opModeIsActive() && (frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy())){}
        /*if(armIsReady) {
            armMotor.setTargetPosition(targetArmPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.1);
        }*/
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware variables
            frontRightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            frontLeftDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            backRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
            backLeftDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
            armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
            wristServo = hardwareMap.get(Servo.class, "wrist_servo");
            leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
            rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");
            touchSensor = hardwareMap.get(TouchSensor.class, "touch");
            telemetry.addData("Status", "Initialized");
            telemetry.update();

        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


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
            case Right:
                runToPosition(STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION);
                runToPosition(-RIGHT_POSITION,RIGHT_POSITION, RIGHT_POSITION, -RIGHT_POSITION);
                break;
            case Left:
                runToPosition(STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION, STRAFE_FORWARD_POSITION);
                runToPosition(LEFT_POSITION, -LEFT_POSITION,-LEFT_POSITION,LEFT_POSITION);
                break;
            case Middle:
                runToPosition(MIDDLE_FORWARD_POSITION, MIDDLE_FORWARD_POSITION, MIDDLE_FORWARD_POSITION, MIDDLE_FORWARD_POSITION);
                break;
        }
        camera.stopStreaming();

        while(opModeIsActive() && (frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy())){

        }
        armMotor.setPower(0);
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

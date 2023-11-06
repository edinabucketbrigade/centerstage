package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;
import com.qualcomm.robotcore.hardware.IMU;

@Config
public class RobotHardwareA {

    /*
    Control Hub Portal
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - arm_motor
                1 - Unspecified Motor - perpendicular_encoder
                2 - Unspecified Motor - parallel_encoder
            Digital Devices
                7 - REV Touch Sensor - touch
        Control Hub
            Motors
                0 - GoBILDA 5201 series - right_back_drive
                1 - GoBILDA 5201 series - right_front_drive
                2 - GoBILDA 5201 series - left_front_drive
                3 - GoBILDA 5201 series - left_back_drive
            Servos
                0 - Servo - right_claw_servo
                1 - Servo - left_claw_servo
                2 - Servo - wrist_servo
    Webcam 1
    */

    public static double LEFT_CLAW_OPEN_POSITION = 0.7;
    public static double LEFT_CLAW_CLOSED_POSITION = 1;
    public static double RIGHT_CLAW_CLOSED_POSITION = 0.275;
    public static double RIGHT_CLAW_OPEN_POSITION = 0.6;
    public static double WRIST_DOWN_POSITION = 0.12;
    public static double WRIST_UP_POSITION = 0.8;
    public static int ARM_DOWN_POSITION = 0;
    public static int ARM_UP_POSITION = 1300;
    public static double ARM_RAISE_POWER = 0.8;
    public static double ARM_LOWER_POWER = 0.4;
    public static double TURTLE_FACTOR = 4;
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 360;
    private static final String TAG = "Bucket Brigade";
    private LinearOpMode opMode;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private IMU imu;
    private Encoder parallelEncoder;
    private Encoder perpendicularEncoder;
    private DcMotor armMotor;
    private Servo wristServo;
    private Servo leftClawServo;
    private Servo rightClawServo;
    private TouchSensor touchSensor;
    private boolean leftClawIsOpen;
    private boolean rightClawIsOpen;
    private boolean wristIsUp;
    private boolean armIsUp;
    private boolean isArmReady;
    private boolean isFieldCentric;
    private boolean isTurtleMode;

    public RobotHardwareA (LinearOpMode opMode) {
        this.opMode = opMode;

        HardwareMap hardwareMap = opMode.hardwareMap;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallel_encoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicular_encoder"));

        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");

        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);

        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FtcDashboard.getInstance();

        Telemetry telemetry = opMode.telemetry;

        telemetry.addData("Status", "Initialized");
    }

    public void toggleLeftClaw() {
        if(leftClawIsOpen) {
            closeLeftClaw();
        }
        else {
            openLeftClaw();
        }
    }

    public void openLeftClaw() {
        log("open left claw");
        leftClawServo.setPosition(LEFT_CLAW_OPEN_POSITION);
        leftClawIsOpen = true;
    }

    public void closeLeftClaw() {
        log("close left claw");
        leftClawServo.setPosition(LEFT_CLAW_CLOSED_POSITION);
        leftClawIsOpen = false;
    }

    public void toggleRightClaw() {
        if(rightClawIsOpen) {
            closeRightClaw();
        }
        else {
            openRightClaw();
        }
    }

    public void openRightClaw() {
        log("open right claw");
        rightClawServo.setPosition(RIGHT_CLAW_OPEN_POSITION);
        rightClawIsOpen = true;
    }

    public void closeRightClaw() {
        log("close right claw");
        rightClawServo.setPosition(RIGHT_CLAW_CLOSED_POSITION);
        rightClawIsOpen = false;
    }

    public void toggleWrist() {
        if(wristIsUp) {
            lowerWrist();
        }
        else {
            raiseWrist();
        }
    }

    public void raiseWrist() {
        log("raise wrist");
        wristServo.setPosition(WRIST_UP_POSITION);
        wristIsUp = true;
    }

    public void lowerWrist() {
        log("lower wrist");
        wristServo.setPosition(WRIST_DOWN_POSITION);
        wristIsUp = false;
    }

    public void moveRobot(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void initializeArm() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void toggleArm() {
        if(armIsUp) {
            lowerArm();
        }
        else {
            raiseArm();
        }
    }

    public void raiseArm() {
        if (!isArmReady) {
            return;
        }
        log("raise arm");
        armMotor.setTargetPosition(ARM_UP_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_RAISE_POWER);
        armIsUp = true;
    }

    public void lowerArm() {
        if (!isArmReady) {
            return;
        }
        log("lower arm");
        armMotor.setTargetPosition(ARM_DOWN_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_LOWER_POWER);
        armIsUp = false;
    }

    public void update() {
        boolean isPressed = touchSensor.isPressed();
        if (isPressed && !isArmReady) {
            initializeArm();
            isArmReady = true;
        }

        Telemetry telemetry = opMode.telemetry;
        telemetry.addData("Status", "Running");
        telemetry.addData("Arm Ready", isArmReady);
        telemetry.addData("Left Front Motor Position/Power", "%d, %.2f", leftFrontDrive.getCurrentPosition(), leftFrontDrive.getPower());
        telemetry.addData("Left Back Motor Position/Power", "%d, %.2f", leftBackDrive.getCurrentPosition(), leftBackDrive.getPower());
        telemetry.addData("Right Front Motor Position/Power", "%d, %.2f", rightFrontDrive.getCurrentPosition(), rightFrontDrive.getPower());
        telemetry.addData("Right Back Motor Position/Power", "%d, %.2f", rightBackDrive.getCurrentPosition(), rightBackDrive.getPower());
        telemetry.addData("Parallel Encoder Position", parallelEncoder.getCurrentPosition());
        telemetry.addData("Perpendicular Encoder Position", perpendicularEncoder.getCurrentPosition());
        telemetry.addData("Arm Motor Position/Power", "%d, %.2f", armMotor.getCurrentPosition(), armMotor.getPower());
        telemetry.addData("Touch Sensor Pressed", touchSensor.isPressed());
        telemetry.addData("Left Claw Servo Position", "%.2f", leftClawServo.getPosition());
        telemetry.addData("Right Claw Servo Position", "%.2f", rightClawServo.getPosition());
        telemetry.addData("Wrist Servo Position", "%.2f", wristServo.getPosition());
        telemetry.addData("Field Centric", isFieldCentric);
        telemetry.addData("Turtle Mode", isTurtleMode);
    }

    public void moveRobot() {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        if (isFieldCentric) {

            double y = -opMode.gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = opMode.gamepad1.left_stick_x;
            double rx = opMode.gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (opMode.gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFrontPower = (rotY + rotX + rx) / denominator;
            leftBackPower = (rotY - rotX + rx) / denominator;
            rightFrontPower = (rotY - rotX - rx) / denominator;
            rightBackPower = (rotY + rotX - rx) / denominator;
        } else {
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -opMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = opMode.gamepad1.left_stick_x;
            double yaw = opMode.gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

        }
        if (isTurtleMode) {
            leftFrontPower /= TURTLE_FACTOR;
            leftBackPower /= TURTLE_FACTOR;
            rightBackPower /= TURTLE_FACTOR;
            rightFrontPower /= TURTLE_FACTOR;
        }
        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.
            /*
            leftBackPower = opMode.gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftFrontPower = opMode.gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightBackPower = opMode.gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightFrontPower  = opMode.gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */
        // Send calculated power to wheels
        moveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void log(String message) {
        Log.d(TAG, message);
    }

    public void runToPosition(int frontRightPosition, int frontLeftPosition, int backRightPosition, int backLeftPosition, double power){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontDrive.setTargetPosition(frontRightPosition);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(power);
        leftFrontDrive.setTargetPosition(frontLeftPosition);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(power);
        leftBackDrive.setTargetPosition(backLeftPosition);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(power);
        rightBackDrive.setTargetPosition(backRightPosition);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(power);

        while(opMode.opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            update();
            opMode.telemetry.update();
        }
    }
    public void setIsFieldCentric(boolean isFieldCentric){
        this.isFieldCentric=isFieldCentric;
    }
    public void toggleFieldCentric(){
        this.isFieldCentric=!isFieldCentric;
    }

    public boolean isArmRaised() {
        int difference = Math.abs(armMotor.getCurrentPosition() - ARM_UP_POSITION);

        if (difference < 50) {
            return true;
        }
        else {
            return false;
        }
    }
    public void setTurtleMode(boolean isTurtleMode){
        this.isTurtleMode = isTurtleMode;
    }

}
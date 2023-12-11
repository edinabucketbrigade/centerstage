package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class RobotHardwareA {

    /*
    Control Hub Portal
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - arm_motor
                1 - Unspecified Motor - perpendicular_encoder
                2 - Unspecified Motor - parallel_encoder
                3 - GoBILDA 5201 series - winch_motor
            Digital Devices
                7 - REV Touch Sensor - touch
            Servos
                0 - Servo - winch_servo
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
                3 - Servo - drone_launch_servo
                4 - Servo - drone_lift_servo
            Digital Devices
                4 - Digital Device - green_light_a
                5 - Digital Device - red_light_a
                6 - Digital Device - green_light_b
                7 - Digital Device - red_light_b
    Webcam 1
    */

    public static double LEFT_CLAW_OPEN_WIDE_POSITION = 0.65;
    public static double LEFT_CLAW_OPEN_NARROW_POSITION = 0.8;
    public static double LEFT_CLAW_CLOSED_POSITION = 1;
    public static double RIGHT_CLAW_CLOSED_POSITION = 0.275;
    public static double RIGHT_CLAW_OPEN_WIDE_POSITION = 0.6;
    public static double RIGHT_CLAW_OPEN_NARROW_POSITION = 0.45;
    public static double WRIST_DOWN_POSITION = 0.08;
    public static double HIGH_WRIST_UP_POSITION = 0.75;
    public static double LOW_WRIST_UP_POSITION = 0.66;
    public static int ARM_DOWN_POSITION = 0;
    public static int HIGH_ARM_UP_POSITION = 1300;
    public static int LOW_ARM_UP_POSITION = 1500;
    public static double ARM_RAISE_POWER = 1;
    public static double ARM_LOWER_POWER = 0.9;
    public static double TURTLE_MULTIPLIER = 0.3;
    public static double NORMAL_MULTIPLIER = 0.6;
    public static double BUNNY_MULTIPLIER = 1;
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 360;
    private static final String TAG = "Bucket Brigade";
    public static double ARM_GAIN = 0.0012;
    public static int ARM_POSITION_THRESHOLD = 50;
    public static double HEADING_THRESHOLD = 1.0;
    public static double TURN_GAIN = 0.03;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public static double WINCH_MOTOR_SPEED = 1;
    public static double WINCH_SERVO_UP_POSITION = 0.72;
    public static double WINCH_SERVO_DOWN_POSITION = 0;
    public static double INITIAL_DRONE_LAUNCH_POSITION = 0.68; // 0.6 - 1
    public static double FINAL_DRONE_LAUNCH_POSITION = 0.82;
    public static double INITIAL_DRONE_LIFT_POSITION = 0.25; // 0.25 - 0.75
    public static double FINAL_DRONE_LIFT_POSITION = 0.41;
    public static double DRONE_INCREMENT = 0.01;
    public LinearOpMode opMode;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor[] driveMotors;
    private IMU imu;
    private Encoder parallelEncoder;
    private Encoder perpendicularEncoder;
    private DcMotor armMotor;
    private Servo wristServo;
    private Servo leftClawServo;
    private Servo rightClawServo;
    private TouchSensor touchSensor;
    //private ColorSensor colorSensor;
    private boolean leftClawIsOpen;
    private boolean rightClawIsOpen;
    private boolean isWristTargetUp;
    private boolean isWinchLiftTargetUp;
    private boolean isArmTargetUp;
    private boolean isArmReady;
    private boolean isFieldCentric = false;
    private boolean isTurtleMode;
    private boolean isBunnyMode = true;
    private double headingError = 0;
    private double  targetHeading = 0;
    /*private DcMotor hookMotor;
    private Servo launcher;*/
    private DigitalChannel greenLightA;
    private DigitalChannel redLightA;
    private DigitalChannel greenLightB;
    private DigitalChannel redLightB;
    private boolean isReverse;
    public boolean isHighDrop;
    private DcMotor winchMotor;
    private Servo winchServo;
    private Servo droneLaunchServo;
    private Servo droneLiftServo;

    public RobotHardwareA (LinearOpMode opMode) {
        this.opMode = opMode;

        HardwareMap hardwareMap = opMode.hardwareMap;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        driveMotors = new DcMotor[] {leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive};

        greenLightA = hardwareMap.get(DigitalChannel.class, "green_light_a");
        redLightA = hardwareMap.get(DigitalChannel.class, "red_light_a");
        greenLightB = hardwareMap.get(DigitalChannel.class, "green_light_b");
        redLightB = hardwareMap.get(DigitalChannel.class, "red_light_b");

        //colorSensor = hardwareMap.colorSensor.get("color");

        winchMotor = hardwareMap.get(DcMotor.class, "winch_motor");

        greenLightA.setMode(DigitalChannel.Mode.OUTPUT);
        redLightA.setMode(DigitalChannel.Mode.OUTPUT);
        greenLightB.setMode(DigitalChannel.Mode.OUTPUT);
        redLightB.setMode(DigitalChannel.Mode.OUTPUT);

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
        winchServo = hardwareMap.get(Servo.class, "winch_servo");

        droneLaunchServo = hardwareMap.get(Servo.class, "drone_launch_servo");
        droneLiftServo = hardwareMap.get(Servo.class, "drone_lift_servo");

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

        winchMotor.setDirection(DcMotor.Direction.FORWARD);
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        droneLaunchServo.setPosition(INITIAL_DRONE_LAUNCH_POSITION);
        droneLiftServo.setPosition(INITIAL_DRONE_LIFT_POSITION);

        /*colorSensor.red();
        colorSensor.green();
        colorSensor.blue();
        colorSensor.alpha();
        colorSensor.argb();*/

        FtcDashboard.getInstance();

        Telemetry telemetry = opMode.telemetry;

        telemetry.addData("Status", "Initialized");
    }

    public void toggleLeftClaw() {
        log("toggle left claw");
        if(leftClawIsOpen) {
            closeLeftClaw();
        }
        else {
            openLeftClaw();
        }
    }

    public void openLeftClaw() {
        log("open left claw");
        if (isReverse) {
            rightClawServo.setPosition(isArmTargetUp ? RIGHT_CLAW_OPEN_NARROW_POSITION : RIGHT_CLAW_OPEN_WIDE_POSITION);
        }
        else {
            leftClawServo.setPosition(isArmTargetUp ? LEFT_CLAW_OPEN_NARROW_POSITION : LEFT_CLAW_OPEN_WIDE_POSITION);
        }
        leftClawIsOpen = true;
    }

    public void closeLeftClaw() {
        log("close left claw");
        if (isReverse) {
            rightClawServo.setPosition(RIGHT_CLAW_CLOSED_POSITION);
        }
        else {
            leftClawServo.setPosition(LEFT_CLAW_CLOSED_POSITION);
        }
        leftClawIsOpen = false;
    }

    public void toggleRightClaw() {
        log("toggle right claw");
        if(rightClawIsOpen) {
            closeRightClaw();
        }
        else {
            openRightClaw();
        }
    }

    public void openRightClaw() {
        log("open right claw");
        if (isReverse) {
            leftClawServo.setPosition(isArmTargetUp ? LEFT_CLAW_OPEN_NARROW_POSITION : LEFT_CLAW_OPEN_WIDE_POSITION);
        }
        else {
            rightClawServo.setPosition(isArmTargetUp ? RIGHT_CLAW_OPEN_NARROW_POSITION : RIGHT_CLAW_OPEN_WIDE_POSITION);
        }
        rightClawIsOpen = true;
    }

    public void closeRightClaw() {
        log("close right claw");
        if (isReverse) {
            leftClawServo.setPosition(LEFT_CLAW_CLOSED_POSITION);
        }
        else {
            rightClawServo.setPosition(RIGHT_CLAW_CLOSED_POSITION);
        }
        rightClawIsOpen = false;
    }

    public void closeClaws() {
        log("close claws");
        closeLeftClaw();
        closeRightClaw();
    }

    public void openClaws() {
        log("open claws");
        openLeftClaw();
        openRightClaw();
    }

    public void toggleClaws() {
        log("toggle claws");
        toggleLeftClaw();
        toggleRightClaw();
    }

    public void toggleWrist() {
        log("toggle wrist");
        if(isWristTargetUp) {
            lowerWrist();
        }
        else {
            raiseWrist();
        }
    }

    public void raiseWrist() {
        log("raise wrist");
        double wristUpPosition = getWristUpPosition();
        wristServo.setPosition(wristUpPosition);
        isWristTargetUp = true;
    }

    public void lowerWrist() {
        log("lower wrist");
        wristServo.setPosition(WRIST_DOWN_POSITION);
        isWristTargetUp = false;
    }

    public void toggleReverse() {
        log("toggle reverse");
        if (isReverse) {
            setForward();
        }
        else {
            setReverse();
        }
    }

    public void toggleTurtleMode() {
        log("toggle turtle mode");
        if (isTurtleMode) {
            setTurtleMode(false);
        }
        else {
            setTurtleMode(true);
        }
    }

    public void toggleBunnyMode() {
        log("toggle bunny mode");
        if (isBunnyMode) {
            setBunnyMode(false);
        }
        else {
            setBunnyMode(true);
        }
    }


    public void setReverse() {
        log("set reverse");
        isReverse = true;
    }

    public void setForward() {
        log("set forward");
        isReverse = false;
    }

    public void moveRobot(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void stopDriveMotors() {
        for(DcMotor driveMotor : driveMotors) {
            driveMotor.setPower(0);
        }
    }

    private void initializeArm() {
        log("initialize arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void toggleArm() {
        log("toggle arm");
        if(isArmTargetUp) {
            lowerArm();
        }
        else {
            raiseArm();
        }
    }

    private int getArmUpPosition() {
        return isHighDrop ? HIGH_ARM_UP_POSITION : LOW_ARM_UP_POSITION;
    }

    private double getWristUpPosition() {
        return isHighDrop ? HIGH_WRIST_UP_POSITION : LOW_WRIST_UP_POSITION;
    }

    public void raiseArm() {
        log("raise arm");
        if (!isArmReady) {
            log("arm is not ready");
            return;
        }
        int armUpPosition = getArmUpPosition();
        armMotor.setTargetPosition(armUpPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        isArmTargetUp = true;
    }

    public void lowerArm() {
        log("lower arm");
        if (!isArmReady) {
            log("arm is not ready");
            return;
        }
        armMotor.setTargetPosition(ARM_DOWN_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_LOWER_POWER);
        isArmTargetUp = false;
        openClaws();
    }

    public void update() {
        boolean isPressed = touchSensor.isPressed();
        if (isPressed && !isArmReady) {
            initializeArm();
            isArmReady = true;
        }

        double armPower;
        if (isArmTargetUp) {
            if (isArmRaised()) {
                armPower = 0;
            }
            else {
                int armUpPosition = getArmUpPosition();
                double positionError = Math.abs(armUpPosition - armMotor.getCurrentPosition());
                armPower = Math.min(positionError * ARM_GAIN, ARM_RAISE_POWER);
            }
        }
        else {
            if (isArmLowered()) {
                armPower = 0;
            }
            else {
                double positionError = Math.abs(ARM_DOWN_POSITION - armMotor.getCurrentPosition());
                armPower = Math.min(positionError * ARM_GAIN, ARM_LOWER_POWER);
            }
        }
        armMotor.setPower(armPower);

        greenLightA.setState(!isReverse);
        redLightA.setState(isReverse);
        greenLightB.setState(!isReverse);
        redLightB.setState(isReverse);

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
        telemetry.addData("Bunny Mode", isBunnyMode);
        telemetry.addData("Reverse", isReverse);
        telemetry.addData("Winch Motor Power", "%.2f", winchMotor.getPower());
        telemetry.addData("Winch Servo Target", "%.2f", winchServo.getPosition());
    }

    public void moveRobot() {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        double axial = -opMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = opMode.gamepad1.left_stick_x;
        double yaw = opMode.gamepad1.right_stick_x;

        if(isReverse) {
            yaw = -yaw;
        }

        if (isFieldCentric) {

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1);
            leftFrontPower = (rotY + rotX + yaw) / denominator;
            leftBackPower = (rotY - rotX + yaw) / denominator;
            rightFrontPower = (rotY - rotX - yaw) / denominator;
            rightBackPower = (rotY + rotX - yaw) / denominator;

        }

        else {

            double max;

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

        double multiplier;

        if (isTurtleMode) {
            multiplier = TURTLE_MULTIPLIER;
        }
        else if (isBunnyMode) {
            multiplier = BUNNY_MULTIPLIER;
        }
        else {
            multiplier = NORMAL_MULTIPLIER;
        }

        leftFrontPower *= multiplier;
        leftBackPower *= multiplier;
        rightBackPower *= multiplier;
        rightFrontPower *= multiplier;

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

        if (isReverse) {
            moveRobot(-leftFrontPower, -rightFrontPower, -leftBackPower, -rightBackPower);
        }
        else {
            moveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
    }

    public void log(String message) {
        Log.d(TAG, message);
    }

    public void runToPosition(int frontRightPosition, int frontLeftPosition, int backRightPosition, int backLeftPosition, double power){

        log("run to position " + frontRightPosition);

        runUsingEncoder(0);

        rightFrontDrive.setTargetPosition(frontRightPosition);
        leftFrontDrive.setTargetPosition(frontLeftPosition);
        leftBackDrive.setTargetPosition(backLeftPosition);
        rightBackDrive.setTargetPosition(backRightPosition);

        runToPosition(power);

        while(opMode.opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            update();
            opMode.telemetry.update();
        }

        runUsingEncoder(0);

    }

    public void runUsingEncoder(double power) {
        for(DcMotor driveMotor : driveMotors) {
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotor.setPower(power);
        }
    }

    private void runToPosition(double power) {
        for(DcMotor driveMotor : driveMotors) {
            driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveMotor.setPower(power);
        }
    }

    public void toggleFieldCentric(){
        /*log("toggle field centric");
        this.isFieldCentric=!isFieldCentric;*/
    }

    public boolean isArmRaised() {
        int armUpPosition = getArmUpPosition();
        int difference = Math.abs(armMotor.getCurrentPosition() - armUpPosition);
        return difference < ARM_POSITION_THRESHOLD;
    }
    public boolean isArmLowered() {
        int difference = Math.abs(armMotor.getCurrentPosition() - ARM_DOWN_POSITION);
        return difference < ARM_POSITION_THRESHOLD;
    }
    public void setTurtleMode(boolean isTurtleMode){
        log("set turtle mode = " + isTurtleMode);
        this.isTurtleMode = isTurtleMode;
    }
    public void setBunnyMode(boolean isBunnyMode){
        log("set bunny mode = " + isBunnyMode);
        this.isBunnyMode = isBunnyMode;
    }

    private double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void turnToHeading(double heading) {

        log("turn to heading " + heading);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, TURN_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            getSteeringCorrection(heading, TURN_GAIN);

            // Pivot in place by applying the turning correction
            double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            moveRobot(0, 0, turn);
            double currentHeading = getHeading();

            opMode.telemetry.addData("Target Heading", targetHeading);
            opMode.telemetry.addData("Heading Error", headingError);
            opMode.telemetry.addData("Turn", turn);
            opMode.telemetry.addData("Current Heading", currentHeading);

            update();
            opMode.telemetry.update();
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        moveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }
    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void resetYaw() {
        log("reset yaw");
        imu.resetYaw();
    }

    public void raiseWinchLift() {
        log("raise winch lift");
        winchServo.setPosition(WINCH_SERVO_UP_POSITION);
        isWinchLiftTargetUp = true;
        raiseWrist();
        closeClaws();
    }

    public void lowerWinchLift() {
        log("lower winch lift");
        winchServo.setPosition(WINCH_SERVO_DOWN_POSITION);
        isWinchLiftTargetUp = false;
    }

    public void toggleWinchLift() {
        log("toggle winch lift");
        if (isWinchLiftTargetUp) {
            lowerWinchLift();
        }
        else {
            raiseWinchLift();
        }
    }

    public void raiseRobot() {
        log("raise robot");
        lowerWinchLift();
        winchMotor.setPower(-WINCH_MOTOR_SPEED);
    }

    public void lowerRobot() {
        log("lower robot");
        winchMotor.setPower(WINCH_MOTOR_SPEED);
    }

    public void stopLiftingRobot() {
        //log("stop lifting robot");
        winchMotor.setPower(0);
    }

    public void incrementDroneLaunchPosition() {
        log("increment drone launch position");
        double position = droneLaunchServo.getPosition() + DRONE_INCREMENT;
        setDroneLaunchPosition(position);
    }

    public void decrementDroneLaunchPosition() {
        log("decrement drone launch position");
        double position = droneLaunchServo.getPosition() - DRONE_INCREMENT;
        setDroneLaunchPosition(position);
    }

    public void setDroneLaunchPosition(double position) {
        log("set drone launch position = " + position);
        droneLaunchServo.setPosition(position);
    }

    public double getDroneLaunchPosition() {
        return droneLaunchServo.getPosition();
    }

    public void incrementDroneLiftPosition() {
        log("increment drone lift position");
        double position = droneLiftServo.getPosition() + DRONE_INCREMENT;
        setDroneLiftPosition(position);
    }

    public void decrementDroneLiftPosition() {
        log("decrement drone launch position");
        double position = droneLiftServo.getPosition() - DRONE_INCREMENT;
        setDroneLiftPosition(position);
    }

    public void setDroneLiftPosition(double position) {
        log("set drone lift position = " + position);
        droneLiftServo.setPosition(position);
    }

    public double getDroneLiftPosition() {
        return droneLiftServo.getPosition();
    }

    public void launchDrone() {
        setDroneLiftPosition(FINAL_DRONE_LIFT_POSITION);
        opMode.sleep(2500);
        setDroneLaunchPosition(FINAL_DRONE_LAUNCH_POSITION);
    }
}
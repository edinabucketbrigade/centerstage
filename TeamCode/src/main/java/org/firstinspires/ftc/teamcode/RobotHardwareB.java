package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class RobotHardwareB {

    /*
    Control Hub Portal
        Control Hub
            Motors
                0 - GoBILDA 5201 series - left_lift_motor
                1 - GoBILDA 5201 series - front_encoder
                2 - GoBILDA 5201 series - left_back_drive (encoder port returns 0 and -1)
                3 - GoBILDA 5201 series - left_front_drive (left encoder)
            Servos
                0 - Servo - left_claw_servo
                1 - Servo - right_claw_servo
                2 - Servo - claw_flip_servo
                3 - Servo - intake_servo
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - roller_motor (right encoder)
                1 - GoBILDA 5201 series - right_lift_motor
                2 - GoBILDA 5201 series - right_front_drive (encoder port has bent pin)
                3 - GoBILDA 5201 series - right_back_drive
            Digital Devices
                3 - REV Touch Sensor - lift_touch
            Servos
                0 - Servo - right_grip_servo
                1 - Servo - elbow_servo
                2 - Servo - left_grip_servo
                3 - Servo - wrist_servo
    Webcam 1
    */

    public static double INITIALIZE_WRIST_POSITION = 0.1;
    public static double GROUND_TRAVERSAL_WRIST_POSITION = 0.1;
    public static double BACKDROP_TRAVERSAL_WRIST_POSITION = 0.05;
    public static double PICKUP_TRAVERSAL_WRIST_POSITION = 0.1;
    public static double NEUTRAL_TRAVERSAL_WRIST_POSITION = 0.9;
    public static double BACKDROP_WRIST_POSITION = 0.4;
    public static double PICKUP_WRIST_POSITION = 0.832;
    public static double NEUTRAL_WRIST_POSITION = 0.45;
    public static double GROUND_WRIST_POSITION = 0.35;
    public static double GROUND_TRAVERSAL_ELBOW_POSITION = 0;
    public static double BACKDROP_TRAVERSAL_ELBOW_POSITION = 0.8;
    public static double PICKUP_TRAVERSAL_ELBOW_POSITION = 0.67;
    public static double NEUTRAL_TRAVERSAL_ELBOW_POSITION = 0.78;
    public static double BACKDROP_ELBOW_POSITION = 0;
    public static double PICKUP_ELBOW_POSITION = 0.715;
    public static double NEUTRAL_ELBOW_POSITION = 0.75;
    public static double GROUND_ELBOW_POSITION = 0.15;
    public static double RIGHT_GRIP_OPEN = 0.36;
    public static double RIGHT_GRIP_CLOSED = 0.43;
    public static double LEFT_GRIP_OPEN = 0.25;
    public static double LEFT_GRIP_CLOSED = 0.38;
    public static double RIGHT_CLAW_OPEN = 0.32;
    public static double RIGHT_CLAW_CLOSED = 0.5;
    public static double LEFT_CLAW_OPEN = 0.71;
    public static double LEFT_CLAW_CLOSED = 0.5;
    public static double CLAW_FLIP_SERVO_DOWN = 0.602;
    public static double CLAW_FLIP_SERVO_UP = 0.47;
    public static int ARM_DELAY = 1000;
    public static double ROLLER_POWER = 0.7;
    public static double LIFT_POWER = 1;
    public static int LIFT_DOWN_POSITION = 0;
    public static double INTAKE_SERVO_UP_POSITION = 1;
    public static double INTAKE_SERVO_DOWN_POSITION = 0.38;
    public static double INTAKE_SERVO_OFFSET_POSITION = 0.45;
    public static final int MINIMUM_COLUMN = 1;
    private static final int MAXIMUM_COLUMN_ODD_ROW = 6;
    private static final int MAXIMUM_COLUMN_EVEN_ROW = 7;
    public static final int MINIMUM_ROW = 1;
    public static final int MAXIMUM_ROW = 11;

    private LinearOpMode opMode;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private boolean rightClawOpen = true;
    private boolean leftClawOpen = true;
    private boolean isTurtleMode = false;
    private DcMotor rollerMotor;
    private Servo leftGripServo;
    private Servo rightGripServo;
    private Servo leftClawServo;
    private Servo rightClawServo;
    private Servo wristServo;
    private Servo elbowServo;
    private Servo clawFlipServo;
    private Servo intakeServo;
    private DcMotor leftLiftMotor;
    private DcMotor rightLiftMotor;
    private TouchSensor liftTouch;
    private boolean isLoweringLift;
    private SampleMecanumDrive drive;
    private AprilTagProcessor aprilTagProcessor;
    private boolean isLocalized;
    private FtcDashboard ftcDashboard;
    private HeatSeekB heatSeek = new HeatSeekB(this);
    private boolean isLeftGripOpen;
    private boolean isRightGripOpen;
    private boolean isClawDown;

    // Initializes this.
    public RobotHardwareB(LinearOpMode opMode) {

        // Remember the op mode.
        this.opMode = opMode;

        // Initialize the FTC dashboard.
        ftcDashboard = FtcDashboard.getInstance();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get hardware.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        rollerMotor = hardwareMap.get(DcMotor.class, "roller_motor");
        leftGripServo = hardwareMap.get(Servo.class, "left_grip_servo");
        rightGripServo = hardwareMap.get(Servo.class, "right_grip_servo");
        leftClawServo = hardwareMap.get(Servo.class,"left_claw_servo");
        rightClawServo = hardwareMap.get(Servo.class,"right_claw_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        elbowServo = hardwareMap.get(Servo.class,"elbow_servo");
        clawFlipServo = hardwareMap.get(Servo.class, "claw_flip_servo");
        intakeServo = hardwareMap.get(Servo.class,"intake_servo");
        leftLiftMotor = hardwareMap.get(DcMotor.class,"left_lift_motor");
        rightLiftMotor = hardwareMap.get(DcMotor.class,"right_lift_motor");
        liftTouch = hardwareMap.get(TouchSensor.class, "lift_touch");

        // Initialize hardware.
        leftLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the drive interface.
        drive = new SampleMecanumDrive(hardwareMap);

        // Get an AprilTag processor.
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Indicate that this is initialized.
        telemetry.addData("Status", "Initialized");

    }

    // Update this.
    public void update() throws InterruptedException {

        // Update the robot.
        drive.update();

        // Update heat seek.
        heatSeek.update();

        // Get a detection.
        AprilTagDetection detection = AutoG.getDetection(aprilTagProcessor);

        // If there is a detection...
        if (detection != null) {

            // Get the telemetry.
            Telemetry telemetry = opMode.telemetry;

            // Get the robot's pose.
            Pose2d pose = AutoG.getRobotPose(detection, telemetry);

            // Update the driver interface.
            drive.setPoseEstimate(pose);

            // Remember that we localized the robot.
            isLocalized = true;

        }

        // Get the robot's pose.
        Pose2d pose = drive.getPoseEstimate();

        // Convert the pose to a string.
        String poseString = AutoF.toString(pose);

        // Determine whether the lift is down.
        boolean isLiftDown = liftTouch.isPressed();

        // Get the lift's position.
        int leftLiftPosition = leftLiftMotor.getCurrentPosition();
        int rightLiftPosition = rightLiftMotor.getCurrentPosition();

        // If we finished lowering the lift...
        if (isLoweringLift && isLiftDown) {

            // Reset the lift.
            resetLift();

            // Remember that we finished lowering the lift.
            isLoweringLift = false;

        }

        // Get the lift's power.
        double leftLiftPower = leftLiftMotor.getPower();
        double rightLiftPower = rightLiftMotor.getPower();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Update the telemetry.
        telemetry.addData("Localized", isLocalized);
        if(isLocalized) {
            telemetry.addData("Pose", poseString);
        }
        telemetry.addData("Lift", "Down %b, Position (%d, %d), Power (%.2f, %.2f)", isLiftDown, leftLiftPosition, rightLiftPosition, leftLiftPower, rightLiftPower);

    }

    // Determines whether the robot is localized.
    public boolean isLocalized() {

        // Return the localized value.
        return isLocalized;

    }

    // Resets the lift.
    private void resetLift() throws InterruptedException {

        // Verify inputs exist.
        if(leftLiftMotor == null) {
            throw new InterruptedException("The left lift motor is missing.");
        }
        if(rightLiftMotor == null) {
            throw new InterruptedException("The right lift motor is missing.");
        }

        // Reset the lift.
        resetLift(leftLiftMotor);
        resetLift(rightLiftMotor);

    }

    // Resets a lift motor.
    private static void resetLift(DcMotor liftMotor) throws InterruptedException {

        // Verify inputs exist.
        if(liftMotor == null) {
            throw new InterruptedException("The lift motor is missing.");
        }

        // Reset the lift motor.
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Moves the robot.
    public void moveRobot() throws InterruptedException {

        Gamepad gamepad1 = opMode.gamepad1;

        double NORMAL_MULTIPLIER = 1;
        double TURTLE_MULTIPLIER = 0.6;

        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

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

        double multiplier;

        if (isTurtleMode) {
            multiplier = TURTLE_MULTIPLIER;
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
        //leftBackPower = gamepad1.x ? 1.0 : 0.0;  // X gamepad
        //leftFrontPower = gamepad1.a ? 1.0 : 0.0;  // A gamepad
        //rightBackPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
        //rightFrontPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

        // Send calculated power to wheels
        moveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

    }

    // Moves the robot.
    public void moveRobot(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) throws InterruptedException {

        // Verify inputs exist.
        if(leftFrontDrive == null) {
            throw new InterruptedException("The left front drive motor is missing.");
        }
        if(rightFrontDrive == null) {
            throw new InterruptedException("The right front drive motor is missing.");
        }
        if(leftBackDrive == null) {
            throw new InterruptedException("The left back drive motor is missing.");
        }
        if(rightBackDrive == null) {
            throw new InterruptedException("The right back drive motor is missing.");
        }

        // Move the robot.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    // Sets the turtle mode value.
    public void setTurtleMode(boolean isTurtleMode){
        this.isTurtleMode = isTurtleMode;
    }

    // Raises the lift.
    public void raiseLift(int position) {

        // Raise the lift.
        setLiftPosition(leftLiftMotor, position);
        setLiftPosition(rightLiftMotor, position);
        isLoweringLift = false;

    }

    // Lowers the lift.
    public void lowerLift() throws InterruptedException {

        // Verify inputs exist.
        if(leftLiftMotor == null) {
            throw new InterruptedException("The left lift motor is missing.");
        }
        if(rightLiftMotor == null) {
            throw new InterruptedException("The right lift motor is missing.");
        }

        // Lower the lift.
        setLiftPosition(leftLiftMotor, LIFT_DOWN_POSITION);
        setLiftPosition(rightLiftMotor, LIFT_DOWN_POSITION);
        isLoweringLift = true;

    }

    // Sets the lift position.
    private void setLiftPosition(DcMotor liftMotor, int position) {

        // Set the lift position.
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(LIFT_POWER);

    }

    // Waits for the user to lower the lift.
    public void waitForLiftDown() throws InterruptedException {

        // Verify inputs exist.
        if(leftLiftMotor == null) {
            throw new InterruptedException("The left lift motor is missing.");
        }
        if(opMode == null) {
            throw new InterruptedException("The op mode is missing.");
        }
        if(rightLiftMotor == null) {
            throw new InterruptedException("The right lift motor is missing.");
        }

        // Get gamepad 1.
        Gamepad gamepad1 = opMode.gamepad1;

        // While the lift is up...
        while (!opMode.isStopRequested() && !liftTouch.isPressed()) {

            // Instruct the user to lower the lift.
            log("Press gamepad 1 back to lower lift...");

            // If the user is pressing back...
            if (gamepad1.back) {

                // Lower the lift.
                leftLiftMotor.setPower(-LIFT_POWER);
                rightLiftMotor.setPower(-LIFT_POWER);

            }

            // Otherwise (if the user is not pressing back)...
            else {

                // Stop the lift.
                leftLiftMotor.setPower(0);
                rightLiftMotor.setPower(0);

            }

        }

        // If stop is requested...
        if(opMode.isStopRequested()) {

            // Exit the method.
            return;

        }

        // Notify the user that the lift is down.
        log("Lift is down");

        // Reset the lift.
        resetLift();

    }

    // Logs a message.
    public void log(String message) {

        // If the op mode is missing...
        if (opMode == null) {

            // Exit the method.
            return;

        }

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Show the message.
        telemetry.addData("Message", message);
        telemetry.update();

    }

    // Initializes the robot.
    public void initializeRobot() throws InterruptedException {

        // Verify inputs exist.
        if(clawFlipServo == null) {
            throw new InterruptedException("The claw flip servo is missing.");
        }
        if(elbowServo == null) {
            throw new InterruptedException("The elbow servo is missing.");
        }
        if(intakeServo == null) {
            throw new InterruptedException("The intake servo is missing.");
        }
        if(leftClawServo == null) {
            throw new InterruptedException("The left claw servo is missing.");
        }
        if(leftGripServo == null) {
            throw new InterruptedException("The left grip servo is missing.");
        }
        if(rightClawServo == null) {
            throw new InterruptedException("The right claw servo is missing.");
        }
        if(rightGripServo == null) {
            throw new InterruptedException("The right grip servo is missing.");
        }
        if(wristServo == null) {
            throw new InterruptedException("The wrist servo is missing.");
        }

        // If stop is requested...
        if(opMode.isStopRequested()) {

            // Exit the method.
            return;

        }

        // Notify the user that we are initializing the robot.
        log("Initializing robot...");

        // Initialize the robot.
        wristServo.setPosition(INITIALIZE_WRIST_POSITION);
        closeClaw();
        offsetIntake();
        opMode.sleep(2000);
        elbowServo.setPosition(NEUTRAL_ELBOW_POSITION);
        opMode.sleep(1000);
        wristServo.setPosition(NEUTRAL_WRIST_POSITION);
        closeGrips();
        lowerClaw();
        opMode.sleep(1000);
        openClaw();
        pointIntakeDown();

        // Notify the user that the robot is initialized.
        log("Initialized robot");

    }

    // Toggles the claws.
    public void toggleClaws() throws InterruptedException {
        toggleLeftClaw();
        toggleRightClaw();
    }

    // Toggles the left claw.
    public void toggleLeftClaw() throws InterruptedException {

        // Verify inputs exist.
        if(leftClawServo == null) {
            throw new InterruptedException("The left claw servo is missing.");
        }

        // Toggle the left claw.
        if (leftClawOpen) {
            leftClawServo.setPosition(LEFT_CLAW_CLOSED);
            leftClawOpen = false;
        } else {
            leftClawServo.setPosition(LEFT_CLAW_OPEN);
            leftClawOpen = true;
        }

    }

    // Toggles the right claw.
    public void toggleRightClaw() throws InterruptedException {

        // Verify inputs exist.
        if(rightClawServo == null) {
            throw new InterruptedException("The right claw servo is missing.");
        }

        // Toggle the right claw.
        if (rightClawOpen) {
            rightClawServo.setPosition(RIGHT_CLAW_CLOSED);
            rightClawOpen = false;
        } else {
            rightClawServo.setPosition(RIGHT_CLAW_OPEN);
            rightClawOpen = true;
        }

    }

    // Powers the roller to intake.
    public void intakeRoller() throws InterruptedException {

        // Verify inputs exist.
        if(rollerMotor == null) {
            throw new InterruptedException("The roller motor is missing.");
        }

        // Power the roller to intake.
        rollerMotor.setPower(-ROLLER_POWER);

    }

    // Powers the roller to eject.
    public void ejectRoller() throws InterruptedException {

        // Verify inputs exist.
        if(rollerMotor == null) {
            throw new InterruptedException("The roller motor is missing.");
        }

        // Power the roller to eject.
        rollerMotor.setPower(ROLLER_POWER);

    }

    // Stops the roller
    public void stopRoller() throws InterruptedException {

        // Verify inputs exist.
        if(rollerMotor == null) {
            throw new InterruptedException("The roller motor is missing.");
        }

        // Stop the roller.
        rollerMotor.setPower(0);

    }

    // Shows the camera stream in the FTC dashboard.
    public void startCameraStream(CameraStreamSource camera) {

        // Show the camera stream in the FTC dashboard.
        ftcDashboard.startCameraStream(camera, 0);

    }

    // Get the drive interface.
    public SampleMecanumDrive getDrive() {

        // Returns the drive interface.
        return drive;

    }

    // Starts looking for AprilTags.
    public void startLookingForAprilTags() {

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Construct a vision portal.
        new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT))
                .addProcessor(aprilTagProcessor)
                .build();

    }

    // Gets the column count for a specified row.
    public static int getMaximumColumn(int row){
        if (isEven(row)){
            return MAXIMUM_COLUMN_EVEN_ROW;
        } else {
            return MAXIMUM_COLUMN_ODD_ROW;
        }
    }

    // Determines whether a number is even.
    public static boolean isEven(int value) {
        if (value % 2 == 0) {
            return true;
        } else {
            return false;
        }
    }

    // Determines whether we are heat seeking.
    public boolean isHeatSeeking() {

        // Return indicating whether we are heat seeking.
        return heatSeek.isActive();

    }

    // Starts heat seeking.
    public void startHeatSeeking(int leftColumn, int row, boolean redAlliance) {

        // Start heat seeking.
        heatSeek.start(leftColumn, row, redAlliance);

    }

    public void offsetIntake() {
        intakeServo.setPosition(INTAKE_SERVO_OFFSET_POSITION);
    }

    public void pointIntakeDown() {
        intakeServo.setPosition(INTAKE_SERVO_DOWN_POSITION);
    }

    public void closeLeftClaw() {
        leftClawServo.setPosition(LEFT_CLAW_CLOSED);
    }

    public void openLeftClaw() {
        leftClawServo.setPosition(LEFT_CLAW_OPEN);
    }

    public void closeRightClaw() {
        rightClawServo.setPosition(RIGHT_CLAW_CLOSED);
    }

    public void openRightClaw() {
        rightClawServo.setPosition(RIGHT_CLAW_OPEN);
    }

    public void lowerClaw() {
        clawFlipServo.setPosition(CLAW_FLIP_SERVO_DOWN);
        isClawDown = true;
    }

    public void raiseClaw() {
        clawFlipServo.setPosition(CLAW_FLIP_SERVO_UP);
        isClawDown = false;
    }

    public void closeLeftGrip() {
        leftGripServo.setPosition(LEFT_GRIP_CLOSED);
        isLeftGripOpen = false;
    }

    public void openLeftGrip() {
        leftGripServo.setPosition(LEFT_GRIP_OPEN);
        isLeftGripOpen = true;
    }

    public void closeClaw() {
        closeLeftClaw();
        closeRightClaw();
    }

    public void openClaw() {
        openLeftClaw();
        openRightClaw();
    }

    public void closeRightGrip() {
        rightGripServo.setPosition(RIGHT_GRIP_CLOSED);
        isRightGripOpen = false;
    }

    public void openRightGrip() {
        rightGripServo.setPosition(RIGHT_GRIP_OPEN);
        isRightGripOpen = true;
    }

    public void openGrips() {
        openLeftGrip();
        openRightGrip();
    }

    public void closeGrips() {
        closeLeftGrip();
        closeRightGrip();
    }

    public void setBackdropArmPosition() {
        elbowServo.setPosition(BACKDROP_ELBOW_POSITION);
        wristServo.setPosition(BACKDROP_WRIST_POSITION);
    }

    public void setPickupArmPosition() {
        elbowServo.setPosition(PICKUP_ELBOW_POSITION);
        wristServo.setPosition(PICKUP_WRIST_POSITION);
    }

    public void setTraversalArmPosition() {
        elbowServo.setPosition(NEUTRAL_TRAVERSAL_ELBOW_POSITION);
        wristServo.setPosition(NEUTRAL_TRAVERSAL_WRIST_POSITION);
    }

    public void setNeutralArmPosition() {
        elbowServo.setPosition(NEUTRAL_ELBOW_POSITION);
        wristServo.setPosition(NEUTRAL_WRIST_POSITION);
    }

    public void setGroundArmPosition() {
        elbowServo.setPosition(GROUND_ELBOW_POSITION);
        wristServo.setPosition(GROUND_WRIST_POSITION);
    }

    public void toggleGrips() {
        if(isLeftGripOpen) {
            closeLeftGrip();
        }
        else {
            openLeftGrip();
        }

        if(isRightGripOpen) {
            closeRightGrip();
        }
        else {
            openRightGrip();
        }
    }

    public void flipClaw() {
        if(isClawDown) {
            raiseClaw();
        }
        else {
            lowerClaw();
        }
    }

}
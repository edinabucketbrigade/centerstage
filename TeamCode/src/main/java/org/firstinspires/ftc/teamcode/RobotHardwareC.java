package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class RobotHardwareC {

    /*
    Control Hub Portal
        Control Hub
            Motors
                0 - GoBILDA 5201 series - left_lift_motor (has left lift encoder)
                1 - GoBILDA 5201 series - front_encoder (is front odometry encoder)
                2 - GoBILDA 5201 series - left_back_drive (encoder port returns 0 and -1)
                3 - GoBILDA 5201 series - left_front_drive (has left odometry encoder)
            Digital Devices
                6 - Digital Device - green_left_led
                7 - Digital Device - red_left_led
            I2C
                2 - REV Color Sensor V3 - right_claw_color
                3 - REV Color Sensor V3 - left_claw_color
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - arm_motor (has arm encoder)
                1 - GoBILDA 5201 series - right_lift_motor (has right lift encoder)
                2 - GoBILDA 5201 series - right_front_drive (encoder port has bent pin)
                3 - GoBILDA 5201 series - right_back_drive (has right odometry encoder)
            Servos
                1 - Servo - wrist_servo
                2 - Servo - right_claw_servo
                3 - Servo - left_claw_servo
            Digital Devices
                0 - Digital Device - green_right_led
                1 - Digital Device - red_right_led
                3 - REV Touch Sensor - lift_touch
                5 - REV Touch Sensor - arm_up_touch
                7 - REV Touch Sensor - arm_down_touch
            I2C
                2 - REV 2m Distance Sensor - right_back_distance
                3 - REV 2m Distance Sensor - left_back_distance
       Webcam 1
    */

    private static final int MAXIMUM_COLUMN_EVEN_ROW = 7;
    private static final int MAXIMUM_COLUMN_ODD_ROW = 6;
    public static final int MINIMUM_COLUMN = 1;
    public static final int MINIMUM_ROW = 1;

    private LinearOpMode opMode;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private boolean isTurtleMode = false;
    private DistanceSensor leftBackDistance;
    private DistanceSensor rightBackDistance;
    private SampleMecanumDrive drive;
    private AprilTagProcessor aprilTagProcessor;
    private boolean isLocalized;
    private FtcDashboard ftcDashboard;
    private HeatSeekC heatSeek = new HeatSeekC(this);
    private Retract retract = new Retract(this);
    private Lift lift;
    private Arm arm;
    private Claw claw;

    // Initializes this.
    public RobotHardwareC(LinearOpMode opMode) {

        // Initialize the FTC dashboard.
        ftcDashboard = FtcDashboard.getInstance();

        // Remember the op mode.
        this.opMode = opMode;

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get hardware.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftBackDistance = hardwareMap.get(Rev2mDistanceSensor.class, "left_back_distance");
        rightBackDistance = hardwareMap.get(Rev2mDistanceSensor.class, "right_back_distance");
        arm = new Arm(this);
        claw = new Claw(this);
        lift = new Lift(this);

        // Get an AprilTag processor.
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Indicate that this is initialized.
        telemetry.addData("Status", "Initialized");

    }

    // Initialize the drive interface.
    public void initializeDrive() {

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Initialize the drive interface.
        drive = new SampleMecanumDrive(hardwareMap);

    }

    // Update this.
    public void update() throws InterruptedException {

        // Update the robot.
        if(drive != null) drive.update();

        // Update heat seek.
        heatSeek.update();

        // Update retract.
        retract.update();

        // Update the hardware.
        arm.update();
        claw.update();
        lift.update();

        // Get a detection.
        AprilTagDetection detection = AutoG.getDetection(aprilTagProcessor);

        // If there is a detection...
        if (detection != null) {

            // Get the telemetry.
            Telemetry telemetry = opMode.telemetry;

            // Get the robot's pose.
            Pose2d pose = AutoG.getRobotPose(detection, telemetry);

            // Update the driver interface.
            if(drive != null) drive.setPoseEstimate(pose);

            // Remember that we localized the robot.
            isLocalized = true;

        }

        // Initialize a pose string.
        String poseString = "";

        // If the drive interface exists...
        if(drive != null) {

            // Get the robot's pose.
            Pose2d pose = drive.getPoseEstimate();

            // Convert the pose to a string.
            poseString = AutoF.toString(pose);

        }

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Get the back distances.
        double leftBackMillimeters = leftBackDistance.getDistance(DistanceUnit.MM);
        double rightBackMillimeters = rightBackDistance.getDistance(DistanceUnit.MM);

        // Update the telemetry.
        telemetry.addData("Status", "Localized = %b, Heat Seeking = %b, Retracting = %b, Turtle Mode = %b", isLocalized, heatSeek.isActive(), retract.isActive(), isTurtleMode);
        if(isLocalized) {
            telemetry.addData("Pose", poseString);
        }
        telemetry.addData("Back Distance", "Left = %.0f mm, Right = %.0f mm", leftBackMillimeters, rightBackMillimeters);

    }

    // Determines whether the robot is localized.
    public boolean isLocalized() {

        // Return the localized value.
        return isLocalized;

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

    // Raises the arm.
    public void raiseArm() {

        // Raise the arm.
        arm.raise();

    }

    // Lowers the arm.
    public void lowerArm() {

        // Lower the arm.
        arm.lower();

    }

    // Logs a message.
    private void log(String message) {

        // If the op mode is missing...
        if (opMode == null) {

            // Exit the method.
            return;

        }

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Show the message.
        Utilities.log(message, telemetry);

    }

    // Initializes the robot.
    public void initializeRobot() {

        // If stop is requested...
        if(opMode.isStopRequested()) {

            // Exit the method.
            return;

        }

        // Notify the user that we are initializing the robot.
        log("Initializing robot...");

        // Open the claw.
        openClawFully();

        // Lower the wrist.
        lowerWrist();

        // Notify the user that the robot is initialized.
        log("Initialized robot");

    }

    // Toggles the claws.
    public void toggleClaws() {

        // Toggle the claws.
        claw.toggle();

    }

    // Toggles the left claw.
    public void toggleLeftClaw() {

        // Toggle the left claw.
        claw.toggleLeft();

    }

    // Toggles the right claw.
    public void toggleRightClaw() throws InterruptedException {

        // Toggle the right claw.
        claw.toggleRight();

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

    // Stops heat seeking.
    public void stopHeatSeeking() {

        // Stop heat seeking.
        heatSeek.stop();

    }

    public boolean isRetracting() {

        // Return indicating whether we are retracting.
        return retract.isActive();

    }

    // Starts retracting.
    public void startRetracting() {

        // Start retracting.
        retract.start();

    }

    // Closes the left claw.
    public void closeLeftClaw() {

        // Close the left claw.
        claw.closeLeft();

    }

    // Opens the left claw.
    public void openLeftClawFully() {

        // Open the left claw.
        claw.openLeftFully();

    }

    // Closes the right claw.
    public void closeRightClaw() {

        // Close the right claw.
        claw.closeRight();

    }

    // Opens the right claw.
    public void openRightClawFully() {

        // Open the right claw.
        claw.openRightFully();

    }

    // Closes the claw.
    public void closeClaw() {

        // Close the claw.
        claw.close();

    }

    // Opens the claw.
    public void openClawFully() {

        // Open the claw.
        claw.openFully();

    }

    public void openClawPartially() {

        // Open the claw.
        claw.openPartially();

    }

    // Toggles the wrist.
    public void toggleWrist() {

        // Toggle the wrist.
        claw.toggleWrist();

    }

    // Raises the wrist.
    public void raiseWrist() {

        // Raise the wrist.
        claw.raiseWrist();

    }

    // Lowers the wrist.
    public void lowerWrist() {

        // Lower the wrist.
        claw.lowerWrist();

    }

    // Raises the lift.
    public void raiseLift(int position) {

        // Raise the lift.
        lift.raise(position);

    }

    // Lowers the lift.
    public void lowerLift() throws InterruptedException {

        // Lower the lift.
        lift.lower();

    }

    // Waits for the user to lower the lift.
    public void waitForLiftDown() throws InterruptedException {

        // Wait for the user to lower the lift.
        lift.waitForDown();

    }

    // Waits for the user to lower the arm.
    public void waitForArmDown() throws InterruptedException {

        // Wait for the user to lower the arm.
        arm.waitForDown();

    }

    // Gets the op mode.
    public LinearOpMode getOpMode() {

        // Return the op mode.
        return opMode;

    }

    // Determines whether the robot is automatically driving.
    public boolean isAutomaticallyDriving() {

        // Return indicating whether the robot is automatically driving.
        return drive == null ? false : drive.isBusy();

    }

    public void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public boolean isArmDown() {
        return arm.isDown();
    }

    public boolean isArmUp() {
        return arm.isUp();
    }

    public boolean isLiftDown() {
        return lift.isDown();
    }

    public boolean isLiftUp() {
        return lift.isUp();
    }

}
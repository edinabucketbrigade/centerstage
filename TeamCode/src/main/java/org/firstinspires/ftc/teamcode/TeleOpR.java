package org.firstinspires.ftc.teamcode;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp
public class TeleOpR extends LinearOpMode {
    /*
    Webcam 1

    Control Hub Portal
        Control Hub
            Motors
                0 - GoBILDA 5201 series - left_slide_motor
                1 - GoBILDA 5201 series - front_encoder
                2 - GoBILDA 5201 series - left_back_drive (encoder port returns 0 and -1)
                3 - GoBILDA 5201 series - left_front_drive (left encoder)
            Servos
                0 - Servo - right_claw_servo
                1 - Servo - left_claw_servo
                2 - Servo - claw_flip_servo
                3 - Servo - intake_servo
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - roller_motor (right encoder)
                1 - GoBILDA 5201 series - right_slide_motor
                2 - GoBILDA 5201 series - right_front_drive (encoder port has bent pin)
                3 - GoBILDA 5201 series - right_back_drive
            Digital Devices
                3 - REV Touch Sensor - lift_touch
            Servos
                0 - Servo - right_grip_servo
                1 - Servo - elbow_servo
                2 - Servo - left_grip_servo
                3 - Servo - wrist_servo
    */
    private static final int MINIMUM_COLUMN = 1;
    private static final int MAXIMUM_COLUMN_ODD_ROW = 6;
    private static final int MAXIMUM_COLUMN_EVEN_ROW = 7;
    private static final int MINIMUM_ROW = 1;
    private static final int MAXIMUM_ROW = 11;
    public static double GROUND_TRAVERSAL_WRIST_POSITION = 0.1;
    public static double BACKDROP_TRAVERSAL_WRIST_POSITION = 0.05;
    public static double PICKUP_TRAVERSAL_WRIST_POSITION = 0.1;
    public static double NEUTRAL_TRAVERSAL_WRIST_POSITION = 0.9;
    public static double BACKDROP_WRIST_POSITION = 0.35;
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
    public static double RIGHT_GRIP_CLOSED = 0.47;
    public static double LEFT_GRIP_OPEN = 0.25;
    public static double LEFT_GRIP_CLOSED = 0.38;
    public static double LEFT_CLAW_OPEN = 0.32;
    public static double LEFT_CLAW_CLOSED = 0.5;
    public static double RIGHT_CLAW_OPEN = 0.71;
    public static double RIGHT_CLAW_CLOSED = 0.5;
    public static double CLAW_FLIP_SERVO_UP = 1;
    public static double CLAW_FLIP_SERVO_DOWN = 0;
    public static int ARM_DELAY = 1000;
    public static double TRIGGER_THRESHOLD = 0.5;
    public static double ROLLER_POWER = 0.7;
    public static double LIFT_POWER = 0.2;
    private static final String TAG = "Bucket Brigade";
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor[] driveMotors;
    private AprilTagProcessor aprilTagProcessor;
    private boolean fromGround;
    private boolean fromBackdrop;
    private boolean fromPickup;
    private boolean fromNeutral;
    private boolean heatSeeking = false;
    private boolean rightGripOpen;
    private boolean leftGripOpen;
    private boolean rightClawOpen;
    private boolean leftClawOpen;
    private boolean clawFlipServoUp;
    private boolean isBunnyMode = true;
    private DcMotor rollerMotor;
    private Servo leftGripServo;
    private Servo rightGripServo;
    private Servo leftClawServo;
    private Servo rightClawServo;
    private Servo wristServo;
    private Servo elbowServo;
    private Servo clawFlipServo;
    private Servo intakeServo;
    private DcMotor leftSlideMotor;
    private DcMotor rightSlideMotor;
    private TouchSensor liftTouch;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the FTC dashboard.
        FtcDashboard.getInstance();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

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
        leftSlideMotor = hardwareMap.get(DcMotor.class,"left_slide_motor");
        rightSlideMotor = hardwareMap.get(DcMotor.class,"right_slide_motor");
        liftTouch = hardwareMap.get(TouchSensor.class, "lift_touch");

        driveMotors = new DcMotor[] {leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive};

        int leftColumn = MINIMUM_COLUMN;
        int leftRow = MINIMUM_ROW;

        // Get an AprilTag processor.
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Get a vision portal.
        new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT))
                .addProcessor(aprilTagProcessor)
                .build();

        // Get a drive.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        elbowServo.setPosition(NEUTRAL_ELBOW_POSITION);
        wristServo.setPosition(NEUTRAL_WRIST_POSITION);
        fromNeutral = true;

        waitForStart();

        boolean localized = false;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            moveRobot();

            drive.update();

            // Get a detection.
            AprilTagDetection detection = AutoG.getDetection(aprilTagProcessor);

            // If there is a detection...
            if (detection != null) {

                // Get the robot's pose.
                Pose2d pose = AutoG.getRobotPose(detection, telemetry);

                // Update the driver interface.
                drive.setPoseEstimate(pose);

                // Remember that we localized the robot.
                localized = true;

            }

            if(currentGamepad1.dpad_down) {
                heatSeeking = true;
            }
            if (currentGamepad1.dpad_up) {
                heatSeeking = false;
            }

            // If we are heat seeking and we know the robot's location...
            if (heatSeeking && localized) {

                // Get the robot's current pose.
                Pose2d currentPose = drive.getPoseEstimate();

                // Construct a target pose.
                //Pose2d targetPose = new Pose2d(50, 35, Math.toRadians(180)); // blue backdrop middle
                Pose2d targetPose = new Pose2d(50, -35, Math.toRadians(180)); // red backdrop middle

                // Construct a trajectory sequence.
                TrajectorySequence sequence = drive.trajectorySequenceBuilder(currentPose)
                        .lineToLinearHeading(targetPose)
                        .build();

                // Execute the trajectory sequence.
                drive.followTrajectorySequence(sequence);

                // Remember that we are done heat seeking.
                heatSeeking = false;

            }

            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                if (leftClawOpen) {
                    leftClawServo.setPosition(LEFT_CLAW_CLOSED);
                    leftClawOpen = false;
                }
                else {
                    leftClawServo.setPosition(LEFT_CLAW_OPEN);
                    leftClawOpen = true;
                }

                if (rightClawOpen) {
                    rightClawServo.setPosition(RIGHT_CLAW_CLOSED);
                    rightClawOpen = false;
                }
                else {
                    rightClawServo.setPosition(RIGHT_CLAW_OPEN);
                    rightClawOpen = true;
                }
            }
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                if (clawFlipServoUp) {
                    clawFlipServo.setPosition(CLAW_FLIP_SERVO_DOWN);
                    clawFlipServoUp = false;
                }
                else {
                    clawFlipServo.setPosition(CLAW_FLIP_SERVO_UP);
                    clawFlipServoUp = true;
                }
            }
            if (currentGamepad1.left_trigger > TRIGGER_THRESHOLD){
                rollerMotor.setPower(-ROLLER_POWER);
            }
            if (currentGamepad1.right_trigger > TRIGGER_THRESHOLD){
                rollerMotor.setPower(ROLLER_POWER);
            }

            if (currentGamepad2.left_stick_y > TRIGGER_THRESHOLD){
                leftSlideMotor.setPower(-LIFT_POWER);
            }
            else if (currentGamepad2.left_stick_y < -TRIGGER_THRESHOLD){
                leftSlideMotor.setPower(LIFT_POWER);
            }
            else {
                leftSlideMotor.setPower(0);
            }

            if (currentGamepad2.right_stick_y > TRIGGER_THRESHOLD){
                rightSlideMotor.setPower(-LIFT_POWER);
            }
            else if (currentGamepad2.right_stick_y < -TRIGGER_THRESHOLD){
                rightSlideMotor.setPower(LIFT_POWER);
            }
            else {
                rightSlideMotor.setPower(0);
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                if (leftGripOpen){
                    leftGripOpen = false;
                    leftGripServo.setPosition(LEFT_GRIP_OPEN);
                } else {
                    leftGripOpen = true;
                    leftGripServo.setPosition(LEFT_GRIP_CLOSED);
                }
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                if (rightGripOpen){
                    rightGripOpen = false;
                    rightGripServo.setPosition(RIGHT_GRIP_OPEN);
                } else {
                    rightGripOpen = true;
                    rightGripServo.setPosition(RIGHT_GRIP_CLOSED);
                }
            }
            // BACKDROP
            if (currentGamepad1.x && !previousGamepad1.x){
                if (fromGround) {
                    elbowServo.setPosition(BACKDROP_ELBOW_POSITION);
                    wristServo.setPosition(BACKDROP_WRIST_POSITION);
                }
                if (fromNeutral) {
                    wristServo.setPosition(NEUTRAL_TRAVERSAL_WRIST_POSITION);
                    elbowServo.setPosition(BACKDROP_ELBOW_POSITION);
                    sleep(1000);
                    wristServo.setPosition(BACKDROP_WRIST_POSITION);
                    fromNeutral = false;
                }
                if (fromPickup) {
                    elbowServo.setPosition(BACKDROP_TRAVERSAL_ELBOW_POSITION);
                    sleep(500);
                    wristServo.setPosition(BACKDROP_TRAVERSAL_WRIST_POSITION);
                    sleep(1000);
                    elbowServo.setPosition(BACKDROP_ELBOW_POSITION);
                    sleep(1000);
                    wristServo.setPosition(BACKDROP_WRIST_POSITION);
                    fromPickup = false;
                }
                fromBackdrop = true;
            }
            // PICKUP
            if (currentGamepad1.b && !previousGamepad1.b){
                if (fromGround) {
                    elbowServo.setPosition(GROUND_TRAVERSAL_ELBOW_POSITION);
                    sleep(1000);
                    wristServo.setPosition(GROUND_TRAVERSAL_WRIST_POSITION);
                    sleep(1000);
                    elbowServo.setPosition(PICKUP_ELBOW_POSITION);
                    sleep(1000);
                    wristServo.setPosition(PICKUP_WRIST_POSITION);
                    fromGround = false;
                }
                if (fromBackdrop) {
                    wristServo.setPosition(BACKDROP_TRAVERSAL_WRIST_POSITION);
                    sleep(1000);
                    elbowServo.setPosition(PICKUP_ELBOW_POSITION);
                    sleep(1000);
                    wristServo.setPosition(PICKUP_WRIST_POSITION);
                    fromBackdrop = false;
                }
                if (fromNeutral) {
                    elbowServo.setPosition(NEUTRAL_TRAVERSAL_ELBOW_POSITION);
                    wristServo.setPosition(NEUTRAL_TRAVERSAL_WRIST_POSITION);
                    sleep(1000);
                    elbowServo.setPosition(PICKUP_ELBOW_POSITION);
                    wristServo.setPosition(PICKUP_WRIST_POSITION);
                    fromNeutral = false;
                }
                fromPickup = true;
            }
            /*if (currentGamepad1.x && !previousGamepad1.x){
                if (leftClawOpen){
                    leftClawOpen = false;
                    leftClawServo.setPosition(0);
                } else {
                    leftClawOpen = true;
                    leftClawServo.setPosition(1);
                }
            }
            if (currentGamepad1.b && !previousGamepad1.b){
                if (rightClawOpen){
                    rightClawOpen = false;
                    rightClawServo.setPosition(0);
                } else {
                    rightClawOpen = true;
                    rightClawServo.setPosition(1);
                }
            }*/
            // NEUTRAL
            if (currentGamepad1.y && !previousGamepad1.y){
                if (fromGround) {
                    elbowServo.setPosition(GROUND_TRAVERSAL_ELBOW_POSITION);
                    sleep(1000);
                    wristServo.setPosition(GROUND_TRAVERSAL_WRIST_POSITION);
                    sleep(1000);
                    elbowServo.setPosition(NEUTRAL_ELBOW_POSITION);
                    sleep(1000);
                    wristServo.setPosition(NEUTRAL_WRIST_POSITION);
                    fromGround = false;
                }
                if (fromBackdrop) {
                    wristServo.setPosition(BACKDROP_TRAVERSAL_WRIST_POSITION);
                    sleep(1000);
                    elbowServo.setPosition(NEUTRAL_ELBOW_POSITION);
                    sleep(1000);
                    wristServo.setPosition(NEUTRAL_WRIST_POSITION);
                    fromBackdrop = false;
                }
                if (fromPickup) {
                    elbowServo.setPosition(NEUTRAL_ELBOW_POSITION);
                    wristServo.setPosition(NEUTRAL_WRIST_POSITION);
                    fromPickup = false;
                }
                fromNeutral = true;
            }
            // GROUND
            if (currentGamepad1.a && !previousGamepad1.a){
                if (fromBackdrop) {
                    elbowServo.setPosition(GROUND_ELBOW_POSITION);
                    wristServo.setPosition(GROUND_WRIST_POSITION);
                    fromBackdrop = false;
                }
                if (fromNeutral) {
                    wristServo.setPosition(NEUTRAL_TRAVERSAL_WRIST_POSITION);
                    sleep(1000);
                    elbowServo.setPosition(GROUND_TRAVERSAL_ELBOW_POSITION);
                    sleep(1000);
                    elbowServo.setPosition(GROUND_ELBOW_POSITION);
                    wristServo.setPosition(GROUND_WRIST_POSITION);
                    fromNeutral = false;
                }
                if (fromPickup) {
                    elbowServo.setPosition(GROUND_TRAVERSAL_ELBOW_POSITION);
                    wristServo.setPosition(GROUND_TRAVERSAL_WRIST_POSITION);
                    sleep(100);
                    elbowServo.setPosition(GROUND_ELBOW_POSITION);
                    wristServo.setPosition(GROUND_WRIST_POSITION);
                    fromPickup = false;
                }
                fromGround = true;
            }
            if (currentGamepad2.b && !previousGamepad2.b){
                int maximumColumn = getMaximumColumn(leftRow);
                leftColumn = Math.min(leftColumn + 1, maximumColumn - 1);
            }
            if (currentGamepad2.x && !previousGamepad2.x){
                leftColumn = Math.max(leftColumn - 1, MINIMUM_COLUMN);
            }
            if (currentGamepad2.y && !previousGamepad2.y){
                leftRow = Math.min(leftRow + 1, MAXIMUM_ROW);
            }
            if (currentGamepad2.a && !previousGamepad2.a){
                leftRow = Math.max(leftRow - 1, MINIMUM_ROW);
            }
            int rightColumn = leftColumn + 1;
            int rightRow = leftRow;

            // Get the hex display.
            String output = getHexDisplay(leftColumn,leftRow,rightColumn,rightRow);

            // Get the robot's pose.
            Pose2d pose = drive.getPoseEstimate();

            // Convert the pose to a string.
            String poseString = AutoF.toString(pose);

            // Determine whether the lift is down.
            boolean isLiftDown = liftTouch.isPressed();

            // Update the telemetry.
            telemetry.addData("Lift Down", isLiftDown);
            telemetry.addData("Localized", localized);
            telemetry.addData("Pose", poseString);
            telemetry.addData("Left Column", leftColumn);
            telemetry.addData("Left Row", leftRow);
            telemetry.addData("Right Column", rightColumn);
            telemetry.addData("Right Row", rightRow);
            telemetry.addData("Output", output);
            telemetry.update();

        }

    }
    public void moveRobot() {
        double BUNNY_MULTIPLIER = 1;
        double NORMAL_MULTIPLIER = 0.6;

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

        if (isBunnyMode) {
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
//        leftBackPower = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//        leftFrontPower = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//        rightBackPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//        rightFrontPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
        // Send calculated power to wheels

        moveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }
    public void setBunnyMode(boolean isBunnyMode){
        log("set bunny mode = " + isBunnyMode);
        this.isBunnyMode = isBunnyMode;
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
    private static int getMaximumColumn(int row){
        if (isEven(row)){
            return MAXIMUM_COLUMN_EVEN_ROW;
        } else {
            return MAXIMUM_COLUMN_ODD_ROW;
        }
    }
    private static boolean isEven(int value){
        if (value%2 == 0){
            return true;
        } else {
            return false;
        }
    }
    private static String getHexDisplay(int leftColumn, int leftRow, int rightColumn, int rightRow){
        String output = "\n";
        for(int row = MAXIMUM_ROW; row >= MINIMUM_ROW; row--) {
            if (!isEven(row)){
                output += "  ";
            }
            int maximumColumn = getMaximumColumn(row);
            for (int column = MINIMUM_COLUMN; column <= maximumColumn; column++) {
                if (column == leftColumn && leftRow == row) {
                    output += "Ⓛ";
                } else if (column == rightColumn && rightRow == row){
                    output += "Ⓡ";
                } else{
                    output += "〇";
                }
            }
            output += "\n";
        }
        return output;
    }
    public void log(String message) {
        Log.d(TAG, message);
    }
}

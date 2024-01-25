package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    */

    /*
    Gamepad 1: Robot Driver

    - left stick = move robot
    - right stick = rotate robot
    - left trigger = roller intake
    - right trigger = roller eject
    - x = toggle left claw
    - b = toggle right claw
    - a = toggle both claws
    - y = hold for turtle mode
    - dpad up = raise lift
    - dpad down = lower lift

    Gamepad 2: Pixel Driver

    - x = move pixels left
    - b = move pixels right
    - a = move pixels down
    - y = move pixels up
    - dpad down = start heat seek
    - dpad up = stop heat seek
     */

    private static final int MINIMUM_COLUMN = 1;
    private static final int MAXIMUM_COLUMN_ODD_ROW = 6;
    private static final int MAXIMUM_COLUMN_EVEN_ROW = 7;
    private static final int MINIMUM_ROW = 1;
    private static final int MAXIMUM_ROW = 11;
    public static double INITIALIZE_WRIST_POSITION = 0.1;
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
    public static double RIGHT_CLAW_OPEN = 0.32;
    public static double RIGHT_CLAW_CLOSED = 0.5;
    public static double LEFT_CLAW_OPEN = 0.71;
    public static double LEFT_CLAW_CLOSED = 0.5;
    public static double CLAW_FLIP_SERVO_DOWN = 1;
    public static double CLAW_FLIP_SERVO_UP = 0;
    public static int ARM_DELAY = 1000;
    public static double TRIGGER_THRESHOLD = 0.5;
    public static double ROLLER_POWER = 0.7;
    public static double LIFT_POWER = 1;
    public static int LIFT_UP_POSITION = 3000;
    public static int LIFT_DOWN_POSITION = 0;
    public static double INTAKE_SERVO_UP_POSITION = 1;
    public static double INTAKE_SERVO_DOWN_POSITION = 0.38;
    public static double INTAKE_SERVO_PLACEMENT_POSITION = 0.45;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
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

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the FTC dashboard.
        FtcDashboard.getInstance();

        // Initialize gamepads.
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

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

        // Initialize left pixel's column and row.
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
        drive = new SampleMecanumDrive(hardwareMap);

        // Wait for the user to lower the lift.
        waitForLiftDown();

        // Initialize the robot.
        initializeRobot();

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Notify the user that we are waiting for start.
        log("Waiting for start...");

        // Wait for start.
        waitForStart();

        // Remember that the robot is not localized.
        boolean localized = false;

        // While the op mode is active...
        while (opModeIsActive()) {

            // Update the gamepads.
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // If we are not heat seeking...
            if(!heatSeeking) {

                // Move the robot.
                moveRobot();

            }

            // Update the drive interface.
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

            // If the pixel driver pressed dpad down...
            if(currentGamepad2.dpad_down) {

                // Start heat seeking.
                heatSeeking = true;

            }

            // If the pixel drive pressed dpad up...
            if (currentGamepad2.dpad_up) {

                // Stop heat seeking.
                heatSeeking = false;

            }

            // If we are heat seeking and we know the robot's location...
            if (heatSeeking && localized) {

                // Heat seek.
                heatSeek();

                // Remember that we are done heat seeking.
                heatSeeking = false;

            }

            if (currentGamepad1.back && !previousGamepad1.back) {
                placePixelsOnBackdrop();
            }

            // If the robot driver pressed x...
            if(currentGamepad1.x && !previousGamepad1.x) {

                // Toggle the left claw.
                toggleLeftClaw();

            }

            // If the robot driver pressed b...
            if(currentGamepad1.b && !previousGamepad1.b) {

                // Toggle the right claw.
                toggleRightClaw();

            }

            // If the robot driver pressed a...
            if(currentGamepad1.a && !previousGamepad1.a) {

                // Toggle the claws.
                toggleClaws();

            }

            // If the robot driver is holding the left trigger...
            if (currentGamepad1.left_trigger > TRIGGER_THRESHOLD) {

                // Power the roller to intake.
                intakeRoller();

            }

            // Otherwise, if the robot driver is hold the right trigger...
            else if (currentGamepad1.right_trigger > TRIGGER_THRESHOLD) {

                // Power the roller to eject.
                ejectRoller();

            }

            // Otherwise (if the robot driver is not holding a trigger)...
            else {

                // Stop the intake.
                stopRoller();

            }

            // If the robot driver pressed dpad down...
            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {

                // Lower the lift.
                lowerLift();

            }

            // If the robot driver pressed dpad up...
            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {

                // Raise the lift.
                raiseLift();

            }

            // If the robot driver is holding y...
            if(currentGamepad1.y) {

                // Enable turtle mode.
                setTurtleMode(true);

            }

            // Otherwise (if the robot driver is not holding y)...
            else {

                // Disable turtle mode.
                setTurtleMode(false);

            }

            // If the pixel driver pressed b...
            if (currentGamepad2.b && !previousGamepad2.b) {

                // Increment the left column.
                int maximumColumn = getMaximumColumn(leftRow);
                leftColumn = Math.min(leftColumn + 1, maximumColumn - 1);

            }

            // If the pixel driver pressed x...
            if (currentGamepad2.x && !previousGamepad2.x) {

                // Decrement the left column.
                leftColumn = Math.max(leftColumn - 1, MINIMUM_COLUMN);

            }

            // If the pixel driver pressed y...
            if (currentGamepad2.y && !previousGamepad2.y){

                // Increment the left row.
                leftRow = Math.min(leftRow + 1, MAXIMUM_ROW);

                // Update the left column if needed.
                int maximumColumn = getMaximumColumn(leftRow);
                if(leftColumn >= maximumColumn) {
                    leftColumn = maximumColumn - 1;
                }

            }

            // If the pixel driver pressed a...
            if (currentGamepad2.a && !previousGamepad2.a){

                // Decrement the left row.
                leftRow = Math.max(leftRow - 1, MINIMUM_ROW);

                // Update the left column if needed.
                int maximumColumn = getMaximumColumn(leftRow);
                if(leftColumn >= maximumColumn) {
                    leftColumn = maximumColumn - 1;
                }

            }

            // Compute the right column and row.
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

            // Update the telemetry.
            telemetry.addData("Lift Down", isLiftDown);
            telemetry.addData("Left Lift Position", leftLiftPosition);
            telemetry.addData("Right Lift Position", rightLiftPosition);
            telemetry.addData("Left Lift Power", leftLiftPower);
            telemetry.addData("Right Lift Power", rightLiftPower);
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
    public void moveRobot() throws InterruptedException {

        // Verify inputs exist.
        if(gamepad1 == null) {
            throw new InterruptedException("The gamepad 1 is missing.");
        }

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
//        leftBackPower = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//        leftFrontPower = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//        rightBackPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//        rightFrontPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
        // Send calculated power to wheels

        moveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }
    public void setTurtleMode(boolean isTurtleMode){
        this.isTurtleMode = isTurtleMode;
    }
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
    private static String getHexDisplay(int leftColumn, int leftRow, int rightColumn, int rightRow) {
        String output = "\n";
        for (int row = MAXIMUM_ROW; row >= MINIMUM_ROW; row--) {
            if (!isEven(row)) {
                output += "  ";
            }
            int maximumColumn = getMaximumColumn(row);
            for (int column = MINIMUM_COLUMN; column <= maximumColumn; column++) {
                if (column == leftColumn && leftRow == row) {
                    output += "⬤"; //"Ⓛ";
                } else if (column == rightColumn && rightRow == row) {
                    output += "⬤"; //"Ⓡ";
                } else {
                    output += "〇";
                }
            }
            output += "\n";
        }
        return output;
    }

    private void log(String message) {

        // If the telemetry is missing...
        if (telemetry == null) {

            // Exit the method.
            return;

        }

        // Show the message.
        telemetry.addData("Message", message);
        telemetry.update();

    }

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

    private static void resetLift(DcMotor liftMotor) throws InterruptedException {

        // Verify inputs exist.
        if(liftMotor == null) {
            throw new InterruptedException("The lift motor is missing.");
        }

        // Reset the lift.
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void raiseLift() {

        // Raise the lift.
        setLiftPosition(leftLiftMotor, LIFT_UP_POSITION);
        setLiftPosition(rightLiftMotor, LIFT_UP_POSITION);
        isLoweringLift = false;

    }

    private void lowerLift() throws InterruptedException {

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

    private void setLiftPosition(DcMotor liftMotor, int position) {

        // Set the lift position.
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(LIFT_POWER);

    }

    // Waits for the user to lower the lift.
    private void waitForLiftDown() throws InterruptedException {

        // Verify inputs exist.
        if(gamepad1 == null) {
            throw new InterruptedException("The gamepad 1 is missing.");
        }
        if(leftLiftMotor == null) {
            throw new InterruptedException("The left lift motor is missing.");
        }
        if(rightLiftMotor == null) {
            throw new InterruptedException("The right lift motor is missing.");
        }

        // While the lift is up...
        while (!isStopRequested() && !liftTouch.isPressed()) {

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
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Notify the user that the lift is down.
        log("Lift is down");

        // Reset the lift.
        resetLift();

    }

    private void initializeRobot() throws InterruptedException {

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
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Notify the user that we are initializing the robot.
        log("Initializing robot...");

        // Initialize the robot.
        wristServo.setPosition(INITIALIZE_WRIST_POSITION);
        leftClawServo.setPosition(LEFT_CLAW_CLOSED);
        rightClawServo.setPosition(RIGHT_CLAW_CLOSED);
        intakeServo.setPosition(INTAKE_SERVO_PLACEMENT_POSITION);
        sleep(2000);
        elbowServo.setPosition(NEUTRAL_ELBOW_POSITION);
        sleep(1000);
        wristServo.setPosition(NEUTRAL_WRIST_POSITION);
        leftGripServo.setPosition(LEFT_GRIP_CLOSED);
        rightGripServo.setPosition(RIGHT_GRIP_CLOSED);
        clawFlipServo.setPosition(CLAW_FLIP_SERVO_DOWN);
        sleep(1000);
        leftClawServo.setPosition(LEFT_CLAW_OPEN);
        rightClawServo.setPosition(RIGHT_CLAW_OPEN);
        intakeServo.setPosition(INTAKE_SERVO_DOWN_POSITION);
        fromNeutral = true;

        // Notify the user that the robot is initialized.
        log("Initialized robot");

    }

    // Drives to the backdrop and places pixels.
    private void heatSeek() throws InterruptedException {

        // Verify inputs exist.
        if(clawFlipServo == null) {
            throw new InterruptedException("The claw flip servo is missing.");
        }
        if (drive == null) {
            throw new InterruptedException("The drive interface is missing.");
        }
        if(elbowServo == null) {
            throw new InterruptedException("The elbow servo is missing.");
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

        // Get the robot's current pose.
        Pose2d currentPose = drive.getPoseEstimate();

        // Construct a target pose.
        //Pose2d targetPose = new Pose2d(50, 35, Math.toRadians(180)); // blue backdrop middle
        Pose2d targetPose = new Pose2d(50, -35, Math.toRadians(180)); // red backdrop middle

        // Construct a trajectory sequence.
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(currentPose)
                .addDisplacementMarker(0, () -> {
                    leftClawServo.setPosition(LEFT_CLAW_CLOSED);
                    rightClawServo.setPosition(RIGHT_CLAW_CLOSED);
                })
                .addDisplacementMarker(0.5, () -> {
                    clawFlipServo.setPosition(CLAW_FLIP_SERVO_UP);
                })
                .addDisplacementMarker(1.5, () -> {
                    elbowServo.setPosition(NEUTRAL_TRAVERSAL_ELBOW_POSITION);
                    wristServo.setPosition(NEUTRAL_TRAVERSAL_WRIST_POSITION);
                })
                .addDisplacementMarker(2.5, () -> {
                    elbowServo.setPosition(PICKUP_ELBOW_POSITION);
                    wristServo.setPosition(PICKUP_WRIST_POSITION);
                    fromNeutral = false;
                })
                .addDisplacementMarker(3.5, () -> {
                    leftClawServo.setPosition(LEFT_CLAW_OPEN);
                    rightClawServo.setPosition(RIGHT_CLAW_OPEN);
                })
                .addDisplacementMarker(4, () -> {
                    leftGripServo.setPosition(LEFT_GRIP_OPEN);
                    rightGripServo.setPosition(RIGHT_GRIP_OPEN);

                    raiseLift();

                    elbowServo.setPosition(BACKDROP_TRAVERSAL_ELBOW_POSITION);
                })
                .addDisplacementMarker(4.5, () -> {
                    wristServo.setPosition(BACKDROP_TRAVERSAL_WRIST_POSITION);
                })
                .addDisplacementMarker(5.5, () -> {
                    elbowServo.setPosition(BACKDROP_ELBOW_POSITION);
                })
                .addDisplacementMarker(6.5, () -> {
                    wristServo.setPosition(BACKDROP_WRIST_POSITION);
                    fromPickup = false;

                    leftGripServo.setPosition(LEFT_GRIP_CLOSED);
                    rightGripServo.setPosition(RIGHT_GRIP_CLOSED);

                    fromBackdrop = true;
                })
                .lineToLinearHeading(targetPose)
                .build();

        // Execute the trajectory sequence.
        drive.followTrajectorySequence(sequence);
    }

    // Toggles the claws.
    private void toggleClaws() throws InterruptedException {
        toggleLeftClaw();
        toggleRightClaw();
    }

    // Toggles the left claw.
    private void toggleLeftClaw() throws InterruptedException {

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
    private void toggleRightClaw() throws InterruptedException {

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

    private void placePixelsOnBackdrop() {
        intakeServo.setPosition(INTAKE_SERVO_PLACEMENT_POSITION);
        leftClawServo.setPosition(LEFT_CLAW_CLOSED);
        rightClawServo.setPosition(RIGHT_CLAW_CLOSED);
        sleep(500);
        clawFlipServo.setPosition(CLAW_FLIP_SERVO_UP);
        sleep(1000);

        elbowServo.setPosition(NEUTRAL_TRAVERSAL_ELBOW_POSITION);
        wristServo.setPosition(NEUTRAL_TRAVERSAL_WRIST_POSITION);
        sleep(1000);
        elbowServo.setPosition(PICKUP_ELBOW_POSITION);
        wristServo.setPosition(PICKUP_WRIST_POSITION);
        fromNeutral = false;
        sleep(1000);

        leftClawServo.setPosition(LEFT_CLAW_OPEN);
        rightClawServo.setPosition(RIGHT_CLAW_OPEN);
        sleep(500);

        leftGripServo.setPosition(LEFT_GRIP_OPEN);
        rightGripServo.setPosition(RIGHT_GRIP_OPEN);

        raiseLift();
        sleep(3000);

        elbowServo.setPosition(BACKDROP_ELBOW_POSITION);
        wristServo.setPosition(BACKDROP_WRIST_POSITION);
        fromPickup = false;
        sleep(1000);

        leftGripServo.setPosition(LEFT_GRIP_CLOSED);
        rightGripServo.setPosition(RIGHT_GRIP_CLOSED);

        fromBackdrop = true;
    }

    // Powers the roller to intake.
    private void intakeRoller() throws InterruptedException {

        // Verify inputs exist.
        if(rollerMotor == null) {
            throw new InterruptedException("The roller motor is missing.");
        }

        // Power the roller to intake.
        rollerMotor.setPower(-ROLLER_POWER);

    }

    // Powers the roller to eject.
    private void ejectRoller() throws InterruptedException {

        // Verify inputs exist.
        if(rollerMotor == null) {
            throw new InterruptedException("The roller motor is missing.");
        }

        // Power the roller to eject.
        rollerMotor.setPower(ROLLER_POWER);

    }

    // Stops the roller
    private void stopRoller() throws InterruptedException {

        // Verify inputs exist.
        if(rollerMotor == null) {
            throw new InterruptedException("The roller motor is missing.");
        }

        // Stop the roller.
        rollerMotor.setPower(0);

    }

}
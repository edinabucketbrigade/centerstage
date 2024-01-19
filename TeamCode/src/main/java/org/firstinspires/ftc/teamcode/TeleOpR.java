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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp
public class TeleOpR extends LinearOpMode {
    private static final int MINIMUM_COLUMN = 1;
    private static final int MAXIMUM_COLUMN_ODD_ROW = 6;
    private static final int MAXIMUM_COLUMN_EVEN_ROW = 7;
    private static final int MINIMUM_ROW = 1;
    private static final int MAXIMUM_ROW = 11;
    private static final String TAG = "Bucket Brigade";
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor[] driveMotors;
    private AprilTagProcessor aprilTagProcessor;
    private boolean heatSeeking = false;
    private boolean rightGripOpen;
    private boolean leftGripOpen;
    private boolean rightClawOpen;
    private boolean leftClawOpen;
    private boolean isBunnyMode = true;
    private DcMotor spin_motor;
    private Servo left_grip_servo;
    private Servo right_grip_servo;
    private Servo left_claw_servo;
    private Servo right_claw_servo;
    private Servo flip_servo;
    public LinearOpMode opMode;

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
        spin_motor = hardwareMap.get(DcMotor.class, "spin_motor");
        left_grip_servo = hardwareMap.get(Servo.class, "left_grip_servo");
        right_grip_servo = hardwareMap.get(Servo.class, "right_grip_servo");
        left_claw_servo = hardwareMap.get(Servo.class,"left_claw_servo");
        right_claw_servo = hardwareMap.get(Servo.class,"right_claw_servo");
        flip_servo = hardwareMap.get(Servo.class, "flip_servo");

        driveMotors = new DcMotor[] {leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive};

        int leftColumn = MINIMUM_COLUMN;
        int leftRow = MINIMUM_ROW;

        telemetry.update();

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

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            moveRobot();

            if(currentGamepad1.dpad_down) {
                heatSeeking = true;
            }
            if (currentGamepad1.dpad_up) {
                heatSeeking = false;
            }
            if (heatSeeking){

                // Get a detection.
                AprilTagDetection detection = AutoG.getDetection(aprilTagProcessor);

                // If there is a detection...
                if (detection != null) {
                    // Get the robot's pose.
                    Pose2d startPose = AutoG.getRobotPose(detection, telemetry);

                    // Set the drive's pose estimate.
                    drive.setPoseEstimate(startPose);

                    // Construct a target pose.
                    Pose2d targetPose = new Pose2d(50, 35, Math.toRadians(180)); // blue backdrop middle
                    //Pose2d targetPose = new Pose2d(50, -35, Math.toRadians(180)); // red backdrop middle

                    // Construct a trajectory sequence.
                    TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                            .lineToLinearHeading(targetPose)
                            .build();

                    // Execute the trajectory sequence.
                    drive.followTrajectorySequence(sequence);
                    // completing heatseek
                    heatSeeking = false;
                }
            }
            if (currentGamepad1.left_trigger > 0.5){
                spin_motor.setPower(-0.7);
            }
            if (currentGamepad1.right_trigger > 0.5){
                spin_motor.setPower(0.7);
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                if (leftGripOpen){
                    leftGripOpen = false;
                    left_grip_servo.setPosition(0);
                } else {
                    leftGripOpen = true;
                    left_grip_servo.setPosition(1);
                }
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                if (rightGripOpen){
                    rightGripOpen = false;
                    right_grip_servo.setPosition(0);
                } else {
                    rightGripOpen = true;
                    right_grip_servo.setPosition(1);
                }
            }
            if (currentGamepad1.x && !previousGamepad1.x){
                if (leftClawOpen){
                    leftClawOpen = false;
                    left_claw_servo.setPosition(0);
                } else {
                    leftClawOpen = true;
                    left_claw_servo.setPosition(1);
                }
            }
            if (currentGamepad1.b && !previousGamepad1.b){
                if (rightClawOpen){
                    rightClawOpen = false;
                    right_claw_servo.setPosition(0);
                } else {
                    rightClawOpen = true;
                    right_claw_servo.setPosition(1);
                }
            }
            if (currentGamepad1.y && !previousGamepad1.y){
                flip_servo.setPosition(1);
            }
            if (currentGamepad1.a && !previousGamepad1.y){
                flip_servo.setPosition(0);
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
            getHexDisplay(leftColumn,leftRow,rightColumn,rightRow);
            telemetry.addData("Left Column", leftColumn);
            telemetry.addData("Left Row", leftRow);
            telemetry.addData("Right Column", rightColumn);
            telemetry.addData("Right Row", rightRow);
            telemetry.update();
        }
    }
    public void moveRobot() {
        double BUNNY_MULTIPLIER = 2;
        double NORMAL_MULTIPLIER = 1;

        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        double axial = -opMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = opMode.gamepad1.left_stick_x;
        double yaw = opMode.gamepad1.right_stick_x;

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

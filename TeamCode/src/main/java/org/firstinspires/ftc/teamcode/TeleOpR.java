package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    private static final int MAXIMUM_COLUMN_ODD_ROW = 5;
    private static final int MAXIMUM_COLUMN_EVEN_ROW = 6;
    private static final int MINIMUM_ROW = 1;
    private static final int MAXIMUM_ROW = 11;
    private AprilTagProcessor aprilTagProcessor;
    private boolean heatSeeking = false;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the FTC dashboard.
        FtcDashboard.getInstance();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

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
            if (currentGamepad2.b && !previousGamepad2.b){
                int maximumColumn = getMaximumColumn(leftRow);
                leftColumn = Math.min(leftColumn + 1, maximumColumn);
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
}

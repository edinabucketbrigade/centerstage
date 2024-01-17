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

    private AprilTagProcessor aprilTagProcessor;
    private boolean heatSeeking = false;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the FTC dashboard.
        FtcDashboard.getInstance();

        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

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
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);
            if(currentGamepad.dpad_down) {
                heatSeeking = true;
            }
            if (currentGamepad.dpad_up) {
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
        }
    }
}

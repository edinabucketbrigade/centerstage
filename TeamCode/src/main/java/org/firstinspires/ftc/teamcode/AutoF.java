package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoF.State.DRIVE_TO_BACKDROP_APPROACH;
import static org.firstinspires.ftc.teamcode.AutoF.State.DRIVE_TO_STACK_APPROACH;
import static org.firstinspires.ftc.teamcode.AutoF.State.DRIVE_TO_BACKDROP_PLACE;
import static org.firstinspires.ftc.teamcode.AutoF.State.DRIVE_TO_STACK_GRAB;
import static org.firstinspires.ftc.teamcode.AutoF.State.GRAB_OFF_STACK;
import static org.firstinspires.ftc.teamcode.AutoF.State.IDLE;
import static org.firstinspires.ftc.teamcode.AutoF.State.DRIVE_TO_SPIKE_MARK;
import static org.firstinspires.ftc.teamcode.AutoF.State.PARK;
import static org.firstinspires.ftc.teamcode.AutoF.State.RAISE_ARM_AND_LIFT;
import static org.firstinspires.ftc.teamcode.AutoF.State.RELEASE_PIXEL_ON_BACKDROP;
import static org.firstinspires.ftc.teamcode.AutoF.State.RETRACT;
import static org.firstinspires.ftc.teamcode.AutoF.State.RETURN_TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.HeatSeekC.getTargetLiftPosition;
import static bucketbrigade.casperlibrary.Objectives.PURPLE;
import static bucketbrigade.casperlibrary.Objectives.PURPLE_YELLOW;
import static bucketbrigade.casperlibrary.Objectives.PURPLE_YELLOW_WHITE;

import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_ACCELERATION;
import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_VELOCITY_FAST;
import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_VELOCITY_SLOW;
import static bucketbrigade.casperlibrary.RobotRoutes.TRACK_WIDTH;
import static bucketbrigade.casperlibrary.RobotRoutes.WHITE_PIXEL_ROW;
import static bucketbrigade.casperlibrary.RobotRoutes.YELLOW_PIXEL_ROW;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToBackdropApproach;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToBackdropPlace;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToSpikeMark;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToStackApproach;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToStackGrab;
import static bucketbrigade.casperlibrary.RobotRoutes.park;
import static bucketbrigade.casperlibrary.RobotRoutes.returnToBackdrop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

import bucketbrigade.casperlibrary.Action;
import bucketbrigade.casperlibrary.BackAction;
import bucketbrigade.casperlibrary.LineToAction;
import bucketbrigade.casperlibrary.LineToLinearHeadingAction;
import bucketbrigade.casperlibrary.Objectives;
import bucketbrigade.casperlibrary.RobotPose;
import bucketbrigade.casperlibrary.RobotRoutes;
import bucketbrigade.casperlibrary.SetReversedAction;
import bucketbrigade.casperlibrary.SetTangentAction;
import bucketbrigade.casperlibrary.SplineToAction;
import bucketbrigade.casperlibrary.SplineToConstantHeadingAction;
import bucketbrigade.casperlibrary.SplineToLinearHeadingAction;
import bucketbrigade.casperlibrary.TeamPropLocation;
import bucketbrigade.casperlibrary.TurnAction;

@Config
@Autonomous(preselectTeleOp = "TeleOpT")
public class AutoF extends LinearOpMode {

    public static final int MILLISECONDS_PER_SECOND = 1000;
    public static final int MINIMUM_DELAY = 0;
    public static final int MAXIMUM_DELAY = 30;
    public static int STACK_LIFT_POSITION = 200;

    public static Boolean redAlliance;
    public static Pose2d currentPose;
    public static boolean lastRanAutonomous;
    private Boolean startClose;
    private Boolean parkLeft;
    private Integer delay;
    private int temporaryDelay = MINIMUM_DELAY;
    private Objectives objectives;
    private OpenCvWebcam camera;
    private boolean startedStreaming;
    private Gamepad previousGamepad = new Gamepad();
    private Gamepad currentGamepad = new Gamepad();
    private TeamPropLocation location;
    private CenterStageCVDetection teamPropDetector;
    private RobotHardwareC robotHardware;

    enum State {IDLE, DRIVE_TO_SPIKE_MARK, DRIVE_TO_BACKDROP_APPROACH, RAISE_ARM_AND_LIFT, DRIVE_TO_BACKDROP_PLACE, RELEASE_PIXEL_ON_BACKDROP, WAIT_FOR_RELEASE, RETRACT, DRIVE_TO_STACK_APPROACH, DRIVE_TO_STACK_GRAB, GRAB_OFF_STACK, RETURN_TO_BACKDROP, PARK}

    private State state = IDLE;
    private ElapsedTime timer = new ElapsedTime();
    private Pose2d lastEnd;
    private boolean placingYellowPixel = true;

    @Override
    public void runOpMode() throws InterruptedException {

        // Clear the red alliance value.
        redAlliance = null;

        // Remember that we last ran autonomous.
        lastRanAutonomous = true;

        // Get the robot hardware.
        robotHardware = new RobotHardwareC(this);

        // Wait for menu selection.
        waitForMenuSelection();

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        //robotHardware.waitForArmDown();

        // Wait for the camera to open.
        waitForCameraOpen();

        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Close the claw.
        robotHardware.closeClaw();

        // Wait a bit.
        sleep(2000);

        // Move the wrist to the backdrop position.
        robotHardware.setWristBackdrop();

        // Notify the user that we are waiting for start.
        log("Waiting for start...");

        // Wait for start.
        waitForStart();

        // Wait for team prop detection.
        waitForTeamPropDetection();

        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Start looking for AprilTags.
        robotHardware.startLookingForAprilTags();

        // Initialize the drive interface.
        robotHardware.initializeDrive();

        // Use the delay.
        sleep(delay * MILLISECONDS_PER_SECOND);

        // Start the state machine.
        setState(DRIVE_TO_SPIKE_MARK);

        // While the op mode is active...
        while (!isStopRequested()) {

            // Update this.
            update();

            // Update the robot hardware.
            robotHardware.update();

            // Display the state.
            telemetry.addData("State", state);

            // Update the telemetry.
            telemetry.update();

        }

    }

    // Updates the state machine.
    public void update() throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Update the robot's current pose.
        currentPose = robotHardware.getPose();

        // Get a backdrop lift position.
        int backdropLiftPosition = placingYellowPixel ? getTargetLiftPosition(YELLOW_PIXEL_ROW) : getTargetLiftPosition(WHITE_PIXEL_ROW);

        // Update the state machine.
        switch (state) {

            case DRIVE_TO_SPIKE_MARK:

                // Get a spike mark trajectory sequence.
                TrajectorySequence spikeMarkTrajectorySequence = getSpikeMarkTrajectorySequence();
                lastEnd = spikeMarkTrajectorySequence.end();

                // Start driving to the spike mark.
                drive.followTrajectorySequenceAsync(spikeMarkTrajectorySequence);

                // Move the wrist to the ground position.
                robotHardware.setWristGround();

                // Advance to the next step.
                setState(DRIVE_TO_BACKDROP_APPROACH);

                break;

            case DRIVE_TO_BACKDROP_APPROACH:

                // If the robot is driving...
                if (drive.isBusy()) {

                    // Exit the method.
                    return;

                }

                // Open the left claw.
                robotHardware.openLeftClaw(true);

                // Raise the wrist so it does not bump the purple pixel when the robot starts moving.
                robotHardware.setWristBackdrop();

                // If we are only placing the purple pixel...
                if (objectives == PURPLE) {

                    // If we started close to the backdrop...
                    if (startClose) {

                        // Park.
                        setState(PARK);

                    }

                    // Otherwise, if we started far from the backdrop...
                    else {

                        // We are done.
                        setState(IDLE);

                    }

                    // Exit the method.
                    return;

                }

                // Get a backdrop approach trajectory sequence.
                TrajectorySequence backdropApproachTrajectorySequence = getBackdropApproachTrajectorySequence();
                lastEnd = backdropApproachTrajectorySequence.end();

                // Start driving to the backdrop approach position.
                drive.followTrajectorySequenceAsync(backdropApproachTrajectorySequence);

                // Advance to the next step.
                setState(RAISE_ARM_AND_LIFT);

                break;

            case RAISE_ARM_AND_LIFT:

                // If the robot is driving...
                if (drive.isBusy()) {

                    // Exit the method.
                    return;

                }

                // Raise the lift.
                robotHardware.setLiftPosition(backdropLiftPosition);

                // Raise the arm.
                robotHardware.raiseArm();

                // Move the wrist to the backdrop position.
                robotHardware.setWristBackdrop();

                // Advance to the next step.
                setState(DRIVE_TO_BACKDROP_PLACE);

                break;

            case DRIVE_TO_BACKDROP_PLACE:

                // If we are waiting for the arm or lift...
                if (!robotHardware.isArmUp() || !robotHardware.isLiftInPosition(backdropLiftPosition)) {

                    // Exit the method.
                    return;

                }

                // Get a backdrop place trajectory sequence.
                TrajectorySequence backdropPlaceTrajectorySequence = getBackdropPlaceTrajectorySequence(placingYellowPixel);
                lastEnd = backdropPlaceTrajectorySequence.end();

                // Start driving to the backdrop place position.
                drive.followTrajectorySequenceAsync(backdropPlaceTrajectorySequence);

                // Advance to the next step.
                setState(RELEASE_PIXEL_ON_BACKDROP);

                break;

            case RELEASE_PIXEL_ON_BACKDROP:

                // If the robot is driving...
                if (drive.isBusy()) {

                    // Exit the method.
                    return;

                }

                // Open the claw partially to release the pixel.
                robotHardware.openClaw(false);

                // Advance to the next step.
                setState(RETRACT);

                break;

            case RETRACT:

                // If we are waiting...
                if (timer.milliseconds() < 500) {

                    // Exit the method.
                    return;

                }

                // Start retracting.
                robotHardware.startRetracting();

                // If are placing white pixels...
                if(placingYellowPixel && objectives == PURPLE_YELLOW_WHITE) {

                    // Drive to the white pixel stack.
                    setState(DRIVE_TO_STACK_APPROACH);

                }

                // Otherwise (if we are not placing white pixels)...
                else {

                    // Park.
                    setState(PARK);

                }

                break;

            case DRIVE_TO_STACK_APPROACH:

                //if (robotHardware.isRetracting()) {
                //    return;
                //}

                // Get a stack approach trajectory sequence.
                TrajectorySequence stackApproachTrajectorySequence = getStackApproachTrajectorySequence();
                lastEnd = stackApproachTrajectorySequence.end();

                // Start driving to the stack approach position.
                drive.followTrajectorySequenceAsync(stackApproachTrajectorySequence);

                // Advance to the next step.
                setState(DRIVE_TO_STACK_GRAB);

                break;

            case DRIVE_TO_STACK_GRAB:

                // If the robot is driving...
                if (drive.isBusy()) {

                    // Exit the method.
                    return;

                }

                // Raise the lift.
                robotHardware.setLiftPosition(STACK_LIFT_POSITION);

                // Get a stack grab trajectory sequence.
                TrajectorySequence stackGrabTrajectorySequence = getStackGrabTrajectorySequence();
                lastEnd = stackGrabTrajectorySequence.end();

                // Start driving to the stack grab position.
                drive.followTrajectorySequenceAsync(stackGrabTrajectorySequence);

                // Advance to the next step.
                setState(GRAB_OFF_STACK);

                break;

            case GRAB_OFF_STACK:

                // If the robot is driving or we are waiting for the lift...
                if (drive.isBusy() || !robotHardware.isLiftInPosition(STACK_LIFT_POSITION)) {

                    // Exit the method.
                    return;

                }

                // Close the left claw to grab white pixels.
                robotHardware.closeLeftClaw();

                // Advance to the next step.
                setState(RETURN_TO_BACKDROP);

                break;

            case RETURN_TO_BACKDROP:

                // If we are waiting...
                if (timer.milliseconds() < 500) {

                    // Exit the method.
                    return;

                }

                // Get a return trajectory sequence.
                TrajectorySequence returnTrajectorySequence = getReturnTrajectorySequence();
                lastEnd = returnTrajectorySequence.end();

                // Start driving to the backdrop.
                drive.followTrajectorySequenceAsync(returnTrajectorySequence);

                // Remember that we are placing white pixels.
                placingYellowPixel = false;

                // Advance to the next step.
                setState(RAISE_ARM_AND_LIFT);

                break;

            case PARK:

                // If the robot is driving...
                if (drive.isBusy()) {

                    // Exit the method.
                    return;

                }

                // Get a park trajectory sequence.
                TrajectorySequence parkTrajectorySequence = getParkTrajectorySequence();
                lastEnd = parkTrajectorySequence.end();

                // Start parking.
                drive.followTrajectorySequenceAsync(parkTrajectorySequence);

                // Advance to the next step.
                setState(IDLE);

                break;

            case IDLE:

                break;

            default:

                throw new InterruptedException("Unrecognized state");

        }

    }

    // Converts a pose to a string.
    public static String toString(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double radians = pose.getHeading();
        double degrees = Math.toDegrees(radians);
        String output = String.format("x %.2f, y %.2f, heading %.2f", x, y, degrees);
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

    // Waits for menu selection.
    private void waitForMenuSelection() {

        // While the op mode is active...
        while (!isStopRequested()) {

            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // If the user has not selected an alliance...
            if (redAlliance == null) {
                telemetry.addData("Alliance", "X = blue, B = red");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    redAlliance = false;
                }
                if (currentGamepad.b && !previousGamepad.b) {
                    redAlliance = true;
                }
            }

            // Otherwise, if the user has not selected a starting location...
            else if (startClose == null) {
                telemetry.addData("Start", "X = close, B = far");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    startClose = true;
                }
                if (currentGamepad.b && !previousGamepad.b) {
                    startClose = false;
                }
            }

            // Otherwise, if the user has not selected a parking location...
            else if (parkLeft == null) {
                telemetry.addData("Park", "X = left, B = right");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    parkLeft = true;
                }
                if (currentGamepad.b && !previousGamepad.b) {
                    parkLeft = false;
                }
            }

            // Otherwise, if the user has not selected a delay...
            else if (delay == null) {
                telemetry.addData("Delay = " + temporaryDelay, "X = ok, Y = increase, A = decrease");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    delay = temporaryDelay;
                }
                else if (currentGamepad.y && !previousGamepad.y) {
                    temporaryDelay = Math.min(temporaryDelay + 1, MAXIMUM_DELAY);
                }
                else if (currentGamepad.a && !previousGamepad.a) {
                    temporaryDelay = Math.max(temporaryDelay - 1, MINIMUM_DELAY);
                }
            }

            // Otherwise, if the user has not selected what to place...
            else if (objectives == null) {
                telemetry.addData("Place", "X = purple/yellow/white, A = purple/yellow, B = purple");
                telemetry.update();
                if (currentGamepad.x && !previousGamepad.x) {
                    objectives = PURPLE_YELLOW_WHITE;
                }
                else if (currentGamepad.a && !previousGamepad.a) {
                    objectives = PURPLE_YELLOW;
                }
                else if (currentGamepad.b && !previousGamepad.b) {
                    objectives = PURPLE;
                }
            }

            // Otherwise (if the user finished making menu selections)...
            else {

                // Stop prompting the user for inputs.
                break;

            }

        }

    }

    // Waits for the camera to open.
    private void waitForCameraOpen() throws InterruptedException {

        // Verify inputs exist.
        if (hardwareMap == null) {
            throw new InterruptedException("The hardware map is missing.");
        }
        if (parkLeft == null) {
            throw new InterruptedException("The park left value is missing.");
        }
        if (redAlliance == null) {
            throw new InterruptedException("The red alliance value is missing.");
        }
        if (startClose == null) {
            throw new InterruptedException("The start close value is missing.");
        }
        if (telemetry == null) {
            throw new InterruptedException("The telemetry is missing.");
        }

        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Initialize the camera.
        log("Initializing camera...");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        teamPropDetector = new CenterStageCVDetection(parkLeft, redAlliance, startClose, telemetry, true, delay, objectives);
        camera.setPipeline(teamPropDetector);
        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.

        // Open the camera.
        log("Opening camera...");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(RobotHardwareA.CAMERA_WIDTH, RobotHardwareA.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                startedStreaming = true;
                log("Opened camera");
            }

            @Override
            public void onError(int errorCode) {
                log("Error opening camera: " + errorCode);
            }
        });

        // Wait for streaming to start.
        while (!isStopRequested() && !startedStreaming) {
            log("Waiting for camera to stream...");
            sleep(50);
        }

        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Show the camera stream in the FTC dashboard.
        robotHardware.startCameraStream(camera);

    }

    // Waits for team prop detection.
    private void waitForTeamPropDetection() throws InterruptedException {

        // Verify inputs exist.
        if (camera == null) {
            throw new InterruptedException("The camera is missing.");
        }
        if (teamPropDetector == null) {
            throw new InterruptedException("The team prop detector is missing.");
        }

        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Wait for team prop detection.
        while (!isStopRequested() && location == null) {
            log("Waiting for location detection...");
            sleep(50);
            location = teamPropDetector.getLocation();
        }

        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Display the location.
        log("Location is " + location);

        // Stop streaming.
        camera.stopStreaming();

        // Close the camera.
        camera.closeCameraDevice();

    }

    // Sets the state.
    private void setState(State state) {

        // Set the state.
        this.state = state;

        // Reset the timer.
        timer.reset();

    }

    // Gets a start pose.
    private Pose2d getStartPose() throws InterruptedException {

        // Verify the inputs exist.
        if (redAlliance == null) {
            throw new InterruptedException("The red alliance value is missing.");
        }
        if (startClose == null) {
            throw new InterruptedException("The start close value is missing.");
        }

        // Get a start pose.
        RobotPose inputStartPose = RobotRoutes.getStartPose(redAlliance, startClose);
        Pose2d outputStartPose = new Pose2d(inputStartPose.x, inputStartPose.y, inputStartPose.heading);

        // Return the result.
        return outputStartPose;

    }

    // Gets a spike mark trajectory sequence.
    private TrajectorySequence getSpikeMarkTrajectorySequence() throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Verify the inputs exist.
        if (drive == null) {
            throw new InterruptedException("The drive interface is missing.");
        }
        if (location == null) {
            throw new InterruptedException("The location is missing.");
        }
        if (redAlliance == null) {
            throw new InterruptedException("The red alliance value is missing.");
        }
        if (startClose == null) {
            throw new InterruptedException("The start close value is missing.");
        }

        // Get a start pose.
        Pose2d startPose = getStartPose();

        // Set the robot's pose.
        robotHardware.setPose(startPose);

        // Set the trajectory sequence's start pose.
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(startPose);

        // Drive to the spike mark.
        applyActions(driveToSpikeMark(redAlliance, startClose, location), trajectorySequenceBuilder, true);

        // Build the trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the result.
        return trajectorySequence;

    }

    // Gets a backdrop place trajectory sequence.
    private TrajectorySequence getBackdropPlaceTrajectorySequence(boolean placingYellowPixel) throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Construct a trajectory sequence builder.
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(lastEnd);

        // Drive to the place.
        applyActions(driveToBackdropPlace(redAlliance, location, placingYellowPixel), trajectorySequenceBuilder, false);

        // Get a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the trajectory sequence.
        return trajectorySequence;

    }

    // Gets a backdrop trajectory sequence.
    private TrajectorySequence getBackdropApproachTrajectorySequence() throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Construct a trajectory sequence builder.
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(lastEnd);

        // Drive to the baackdrop.
        applyActions(driveToBackdropApproach(redAlliance, startClose, location), trajectorySequenceBuilder, true);

        // Get a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the trajectory sequence.
        return trajectorySequence;

    }

    // Gets a stack approach trajectory sequence.
    private TrajectorySequence getStackApproachTrajectorySequence() throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Construct a trajectory sequence builder.
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(lastEnd);

        // Drive to the stack approach position.
        applyActions(driveToStackApproach(redAlliance), trajectorySequenceBuilder, true);

        // Construct a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the result.
        return trajectorySequence;

    }

    // Gets a stack grab trajectory sequence.
    private TrajectorySequence getStackGrabTrajectorySequence() throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Construct a trajectory sequence builder.
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(lastEnd);

        // Drive to the stack grab position.
        applyActions(driveToStackGrab(redAlliance), trajectorySequenceBuilder, false);

        // Construct a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the result.
        return trajectorySequence;

    }

    private TrajectorySequence getParkTrajectorySequence() throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Construct a trajectory sequence builder.
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(lastEnd);

        // Park.
        applyActions(park(redAlliance, parkLeft), trajectorySequenceBuilder, true);

        // Get a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the result.
        return trajectorySequence;

    }

    private TrajectorySequence getReturnTrajectorySequence() throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Construct a trajectory sequence builder.
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(lastEnd);

        // Return to backdrop.
        applyActions(returnToBackdrop(redAlliance), trajectorySequenceBuilder, true);

        // Construct a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the result.
        return trajectorySequence;

    }

    public static void applyActions(List<Action> actions, TrajectorySequenceBuilder trajectorySequenceBuilder, boolean fast) throws InterruptedException {

        // Get a maximum velocity.
        double maximumVelocity = fast ? MAXIMUM_VELOCITY_FAST : MAXIMUM_VELOCITY_SLOW;

        // Construct a velocity constraint.
        TrajectoryVelocityConstraint velocityConstraint = new MecanumVelocityConstraint(maximumVelocity, TRACK_WIDTH);

        // Construct an acceleration constraint.
        TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(MAXIMUM_ACCELERATION);

        // Apply the constraints.
        trajectorySequenceBuilder.setConstraints(velocityConstraint, accelerationConstraint);

        // Apply the actions.
        for(Action inputAction : actions) {
            if(inputAction instanceof BackAction) {
                BackAction outputAction = (BackAction)inputAction;
                trajectorySequenceBuilder.back(outputAction.distance);
            }
            else if(inputAction instanceof LineToAction) {
                LineToAction outputAction = (LineToAction)inputAction;
                trajectorySequenceBuilder.lineTo(new Vector2d(outputAction.x, outputAction.y));
            }
            else if(inputAction instanceof LineToLinearHeadingAction) {
                LineToLinearHeadingAction outputAction = (LineToLinearHeadingAction)inputAction;
                trajectorySequenceBuilder.lineToLinearHeading(new Pose2d(outputAction.x, outputAction.y, outputAction.heading));
            }
            else if(inputAction instanceof SetReversedAction) {
                SetReversedAction outputAction = (SetReversedAction)inputAction;
                trajectorySequenceBuilder.setReversed(outputAction.reversed);
            }
            else if(inputAction instanceof SetTangentAction) {
                SetTangentAction outputAction = (SetTangentAction)inputAction;
                trajectorySequenceBuilder.setTangent(outputAction.tangent);
            }
            else if(inputAction instanceof SplineToAction) {
                SplineToAction outputAction = (SplineToAction)inputAction;
                trajectorySequenceBuilder.splineTo(new Vector2d(outputAction.x, outputAction.y), outputAction.heading);
            }
            else if(inputAction instanceof SplineToConstantHeadingAction) {
                SplineToConstantHeadingAction outputAction = (SplineToConstantHeadingAction)inputAction;
                trajectorySequenceBuilder.splineToConstantHeading(new Vector2d(outputAction.x, outputAction.y), outputAction.heading);
            }
            else if(inputAction instanceof SplineToLinearHeadingAction) {
                SplineToLinearHeadingAction outputAction = (SplineToLinearHeadingAction)inputAction;
                trajectorySequenceBuilder.splineToLinearHeading(new Pose2d(outputAction.x, outputAction.y, outputAction.heading), outputAction.tangent);
            }
            else if(inputAction instanceof TurnAction) {
                TurnAction outputAction = (TurnAction)inputAction;
                trajectorySequenceBuilder.turn(outputAction.angle);
            }
            else {
                throw new InterruptedException("The action type is unrecognized.");
            }
        }

    }

}
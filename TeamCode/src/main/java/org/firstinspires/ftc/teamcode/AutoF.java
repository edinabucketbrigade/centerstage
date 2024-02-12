package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoF.State.APPROACH_BACKDROP;
import static org.firstinspires.ftc.teamcode.AutoF.State.DRIVE_TO_PIXEL_STACK;
import static org.firstinspires.ftc.teamcode.AutoF.State.DRIVE_TO_PLACE_POSITION;
import static org.firstinspires.ftc.teamcode.AutoF.State.GRAB_OFF_STACK;
import static org.firstinspires.ftc.teamcode.AutoF.State.IDLE;
import static org.firstinspires.ftc.teamcode.AutoF.State.DRIVE_TO_SPIKE_MARK;
import static org.firstinspires.ftc.teamcode.AutoF.State.PARK;
import static org.firstinspires.ftc.teamcode.AutoF.State.RAISE_ARM_AND_LIFT;
import static org.firstinspires.ftc.teamcode.AutoF.State.RAISE_WRIST;
import static org.firstinspires.ftc.teamcode.AutoF.State.RELEASE_PURPLE_PIXEL;
import static org.firstinspires.ftc.teamcode.AutoF.State.RELEASE_WRIST;
import static org.firstinspires.ftc.teamcode.AutoF.State.RELEASE_YELLOW_PIXEL;
import static org.firstinspires.ftc.teamcode.AutoF.State.RETRACT;
import static org.firstinspires.ftc.teamcode.AutoF.State.RETURN_TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.AutoF.State.WAIT_FOR_RELEASE;

import static bucketbrigade.casperlibrary.RobotRoutes.driveToBackdrop;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToSpikeMark;
import static bucketbrigade.casperlibrary.RobotRoutes.driveToStack;
import static bucketbrigade.casperlibrary.RobotRoutes.park;
import static bucketbrigade.casperlibrary.RobotRoutes.returnToBackdrop;
import static bucketbrigade.casperlibrary.TeamPropLocation.LEFT;
import static bucketbrigade.casperlibrary.TeamPropLocation.MIDDLE;
import static bucketbrigade.casperlibrary.TeamPropLocation.RIGHT;

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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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

    public static final int FIRST_ROW = 1;
    public static final double BACKDROP_TARGET_X = 40;

    public static Boolean redAlliance;
    public static Pose2d currentPose;
    public static boolean lastRanAutonomous;
    private Boolean startClose;
    private Boolean parkLeft;
    private OpenCvWebcam camera;
    private boolean startedStreaming;
    private Gamepad previousGamepad = new Gamepad();
    private Gamepad currentGamepad = new Gamepad();
    private TeamPropLocation location;
    private CenterStageCVDetection teamPropDetector;
    private RobotHardwareC robotHardware;

    enum State {IDLE, DRIVE_TO_SPIKE_MARK, RELEASE_PURPLE_PIXEL, RAISE_WRIST, APPROACH_BACKDROP, RAISE_ARM_AND_LIFT, DRIVE_TO_PLACE_POSITION, RELEASE_YELLOW_PIXEL, RELEASE_WRIST, WAIT_FOR_RELEASE, RETRACT, DRIVE_TO_PIXEL_STACK, GRAB_OFF_STACK, RETURN_TO_BACKDROP, PARK}

    private State state = IDLE;
    private ElapsedTime timer = new ElapsedTime();
    private Pose2d lastEnd;

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

        // Start the state machine.
        setState(DRIVE_TO_SPIKE_MARK);

        // While the op mode is active...
        while (!isStopRequested()) {

            // Update this.
            update();

            // Update the robot hardware.
            robotHardware.update();

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

        // Get a lift position.
        int liftPosition = HeatSeekC.getTargetLiftPosition(FIRST_ROW);

        // Get the appropriate left column.
        int leftColumn = getLeftColumn(location);

        // Get a target y coordinate.
        double targetY = HeatSeekC.getTargetY(leftColumn, FIRST_ROW, redAlliance);

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
                setState(RELEASE_PURPLE_PIXEL);

                break;

            case RELEASE_PURPLE_PIXEL:

                // If the robot is driving...
                if (drive.isBusy()) {

                    // Exit the method.
                    return;

                }

                // Open the left claw.
                robotHardware.openLeftClaw(true);

                // Advance to the next step.
                setState(RAISE_WRIST);

                break;

            case RAISE_WRIST:

                // If we are waiting...
                if (timer.milliseconds() > 500) {

                    // Exit the method.
                    return;

                }

                // Raise the wrist so it does not bump the purple pixel when the robot starts moving.
                robotHardware.setWristBackdrop();

                // Advance to the next step.
                setState(APPROACH_BACKDROP);

                break;

            case APPROACH_BACKDROP:

                if (timer.milliseconds() > 500) {
                    return;
                }

                // Construct an approach trajectory sequence.
                TrajectorySequence approachTrajectorySequence = getBackdropTrajectorySequence(targetY);
                lastEnd = approachTrajectorySequence.end();

                // Start driving to the backdrop.
                drive.followTrajectorySequenceAsync(approachTrajectorySequence);

                // Advance to the next step.
                setState(RAISE_ARM_AND_LIFT);

                break;

            case RAISE_ARM_AND_LIFT:

                if (drive.isBusy()) {
                    return;
                }

                // Raise the lift.
                robotHardware.setLiftPosition(liftPosition);

                // Raise the arm.
                robotHardware.raiseArm();

                setState(DRIVE_TO_PLACE_POSITION);

                break;

            case DRIVE_TO_PLACE_POSITION:

                // If we are waiting...
                if (!robotHardware.isArmUp() || !robotHardware.isLiftInPosition(liftPosition)) {

                    // Exit the method.
                    return;

                }

                // Construct a velocity constraint.
                TrajectoryVelocityConstraint placeVelocityConstraint = new MecanumVelocityConstraint(HeatSeekC.PLACE_SPEED, DriveConstants.TRACK_WIDTH);

                // Construct an acceleration constraint.
                TrajectoryAccelerationConstraint placeAccelerationConstraint = new ProfileAccelerationConstraint(HeatSeekC.PLACE_SPEED);

                // Construct a target position.
                Vector2d targetPosition = new Vector2d(HeatSeekC.PLACE_TARGET_X, targetY);

                // Construct a target pose.
                Pose2d targetPose = new Pose2d(targetPosition, Math.toRadians(180));

                // Construct a trajectory sequence.
                TrajectorySequence placeTrajectorySequence = drive.trajectorySequenceBuilder(lastEnd)
                        .setConstraints(placeVelocityConstraint, placeAccelerationConstraint)
                        .lineToLinearHeading(targetPose)
                        .build();
                lastEnd = placeTrajectorySequence.end();

                // Execute the trajectory sequence.
                drive.followTrajectorySequenceAsync(placeTrajectorySequence);

                setState(RELEASE_YELLOW_PIXEL);

                break;

            case RELEASE_YELLOW_PIXEL:

                if (drive.isBusy()) {
                    return;
                }

                robotHardware.openClaw(false);

                setState(RELEASE_WRIST);

                break;

            case RELEASE_WRIST:

                // If we are waiting...
                if (timer.milliseconds() < 500) {

                    // Exit the method.
                    return;

                }

                // Set the wrist to the release position.
                robotHardware.setWristRelease();

                //Advance to the next step.
                setState(WAIT_FOR_RELEASE);

                break;

            case WAIT_FOR_RELEASE:

                if (timer.milliseconds() < 500) {
                    return;
                }

                setState(RETRACT);

                break;

            case RETRACT:

                // Start retracting.
                robotHardware.startRetracting();

                //setState(DRIVE_TO_PIXEL_STACK);
                setState(PARK);

                break;

            case DRIVE_TO_PIXEL_STACK:

                TrajectorySequence stackTrajectorySequence = getStackTrajectorySequence();
                lastEnd = stackTrajectorySequence.end();

                drive.followTrajectorySequenceAsync(stackTrajectorySequence);

                setState(GRAB_OFF_STACK);

                break;

            case GRAB_OFF_STACK:

                if (drive.isBusy()) {
                    return;
                }

                robotHardware.closeLeftClaw();

                setState(RETURN_TO_BACKDROP);

                break;

            case RETURN_TO_BACKDROP:

                TrajectorySequence returnTrajectorySequence = getReturnTrajectorySequence();
                lastEnd = returnTrajectorySequence.end();

                drive.followTrajectorySequenceAsync(returnTrajectorySequence);

                setState(PARK);

                break;

            case PARK:

                if (drive.isBusy()) {
                    return;
                }

                TrajectorySequence parkTrajectorySequence = getParkTrajectorySequence();
                lastEnd = parkTrajectorySequence.end();

                drive.followTrajectorySequenceAsync(parkTrajectorySequence);

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
        teamPropDetector = new CenterStageCVDetection(parkLeft, redAlliance, startClose, telemetry, true);
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
        applyActions(driveToSpikeMark(redAlliance, startClose, location), trajectorySequenceBuilder);

        // Build the trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the result.
        return trajectorySequence;

    }

    // Gets a backdrop trajectory sequence.
    private TrajectorySequence getBackdropTrajectorySequence(double targetY) throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Construct a trajectory sequence builder.
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(lastEnd);

        // Drive to the baackdrop.
        applyActions(driveToBackdrop(redAlliance, startClose, location, BACKDROP_TARGET_X, targetY), trajectorySequenceBuilder);

        // Get a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the trajectory sequence.
        return trajectorySequence;

    }

    // Gets a stack trajectory sequence.
    private TrajectorySequence getStackTrajectorySequence() throws InterruptedException {

        // Get a drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Construct a trajectory sequence builder.
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(lastEnd);

        // Drive to the stack.
        applyActions(driveToStack(redAlliance), trajectorySequenceBuilder);

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
        applyActions(park(redAlliance, parkLeft), trajectorySequenceBuilder);

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

        // Get the appropriate left column.
        int leftColumn = getLeftColumn(location);

        // Get a target y coordinate.
        double targetY = HeatSeekC.getTargetY(leftColumn, FIRST_ROW, redAlliance);

        // Return to backdrop.
        applyActions(returnToBackdrop(redAlliance, BACKDROP_TARGET_X, targetY), trajectorySequenceBuilder);

        // Construct a trajectory sequence.
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder.build();

        // Return the result.
        return trajectorySequence;

    }

    public static int getLeftColumn(TeamPropLocation location) throws InterruptedException {
        if (location == LEFT) {
            return 1;
        } else if (location == MIDDLE) {
            return 3;
        } else if (location == RIGHT) {
            return 5;
        } else {
            throw new InterruptedException("The location is missing.");
        }
    }

    public static void applyActions(List<Action> actions, TrajectorySequenceBuilder trajectorySequenceBuilder) throws InterruptedException {
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
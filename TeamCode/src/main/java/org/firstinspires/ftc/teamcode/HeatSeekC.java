package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HeatSeekC.State.IDLE;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.CLOSE_CLAW;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.DRIVE_TO_APPROACH_POSITION;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.DRIVE_TO_PLACE_POSITION;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.OPEN_CLAW;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.RELEASE_WRIST;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.WAIT_FOR_RELEASE;
import static org.firstinspires.ftc.teamcode.Lift.MAXIMUM_POSITION;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.MINIMUM_COLUMN;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.MINIMUM_ROW;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.getMaximumColumn;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.isEven;

import static bucketbrigade.casperlibrary.RobotRoutes.PLACE_TARGET_X;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
public class HeatSeekC {

    enum State { IDLE, CLOSE_CLAW, RAISE_ARM_LIFT_AND_WRIST, DRIVE_TO_APPROACH_POSITION, DRIVE_TO_PLACE_POSITION, OPEN_CLAW, RELEASE_WRIST, WAIT_FOR_RELEASE }

    public static double APPROACH_TARGET_X = PLACE_TARGET_X - 10;
    public static int FIRST_ROW_LIFT_POSITION = 0;
    public static int LIFT_INCREMENT = 200;
    public static double PIXEL_WIDTH = 3.2;
    public static double TARGET_RED_Y = -28;
    public static double TARGET_BLUE_Y = 43;
    public static double APPROACH_SPEED = 50;
    public static double PLACE_SPEED = 20;
    public static int MAXIMUM_ROW = (int)Math.floor((MAXIMUM_POSITION - FIRST_ROW_LIFT_POSITION) / LIFT_INCREMENT);

    private RobotHardwareC robotHardware;
    private State state = IDLE;
    private ElapsedTime timer = new ElapsedTime();
    private int leftColumn;
    private int row;
    private boolean redAlliance;

    public HeatSeekC(RobotHardwareC robotHardware) {
        this.robotHardware = robotHardware;
    }

    // Starts heat seeking.
    public void start(int leftColumn, int row, boolean redAlliance) {

        // If we are already heat seeking...
        if(state != IDLE) {

            // Exit the method.
            return;

        }

        // Stop the drive motors.
        robotHardware.stopDriveMotors();

        // Remember the inputs.
        this.leftColumn = leftColumn;
        this.row = row;
        this.redAlliance = redAlliance;

        // Start heat seeking.
        setState(CLOSE_CLAW);

    }

    // Stops heat seeking.
    public void stop() {

        // Get the drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Cancel the trajectory sequence.
        drive.breakFollowing();

        // Stop heat seeking.
        setState(IDLE);

    }

    // Updates this.
    public void update() throws InterruptedException {

        // Get the drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Get a lift position.
        int liftPosition = getTargetLiftPosition(row);

        // Switch based on the state.
        switch (state) {

            case CLOSE_CLAW:

                // Close the claw so it does not catch when raising the lift.
                robotHardware.closeClaw();

                // Advance to the next step.
                setState(DRIVE_TO_APPROACH_POSITION);

                break;

            case DRIVE_TO_APPROACH_POSITION:

                // If we are waiting...
                if (timer.milliseconds() < 500) {

                    // Exit the method.
                    return;

                }

                // Raise the lift.
                robotHardware.setLiftPosition(liftPosition);

                // Set the wrist to the backdrop position.
                robotHardware.setWristBackdrop();

                // Raise the arm.
                robotHardware.raiseArm();

                // Drive to the approach position.
                startDrivingToBackdrop(APPROACH_SPEED, APPROACH_TARGET_X);

                // Advance to the next step.
                setState(DRIVE_TO_PLACE_POSITION);

                break;

            case DRIVE_TO_PLACE_POSITION:

                // If we are waiting...
                if (drive.isBusy() || !robotHardware.isArmUp() || !robotHardware.isLiftInPosition(liftPosition)) {

                    // Exit the method.
                    return;

                }

                // Drive to the place position.
                startDrivingToBackdrop(PLACE_SPEED, PLACE_TARGET_X);

                // Advance to the next step.
                setState(OPEN_CLAW);

                break;

            case OPEN_CLAW:

                // If we are waiting...
                if (drive.isBusy()) {

                    // Exit the method.
                    return;

                }

                // Open the claw partially to release the pixels.
                robotHardware.openClaw(false);

                // Advance to the next step.
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

                setState(IDLE);

                break;

            case IDLE:

                break;

            default:

                throw new InterruptedException("Unrecognized state");

        }

    }

    // Sets the state.
    private void setState(State state) {

        // Set the state.
        this.state = state;

        // Reset the timer.
        timer.reset();

    }

    // Get a target y coordinate.
    public static double getTargetY(int leftColumn, int row, boolean redAlliance) throws InterruptedException {

        // If the row is invalid...
        if(row < MINIMUM_ROW || row > MAXIMUM_ROW) {

            // Complain.
            throw new InterruptedException("The row is invalid.");

        }

        // Get the row's column count.
        int maximumColumn = getMaximumColumn(row);

        // If the left column is invalid...
        if(leftColumn < MINIMUM_COLUMN || leftColumn > maximumColumn - 1) {

            // Complain.
            throw new InterruptedException("The left column is invalid.");

        }

        // Initialize a target y coordinate.
        double targetY = redAlliance ? TARGET_RED_Y : TARGET_BLUE_Y;

        // Determine whether this is an even row.
        boolean isEvenRow = isEven(row);

        // If this is an even row...
        if(isEvenRow) {

            // Shift the robot up by half a pixel width.
            targetY += PIXEL_WIDTH / 2;

        }

        // Shift the robot down to the appropriate column.
        targetY -= (leftColumn - 1) * PIXEL_WIDTH;

        // Return the result.
        return targetY;

    }

    // Gets a target lift position.
    public static int getTargetLiftPosition(int row) {

        // Get a heat seek lift position.
        int position =  Math.min(FIRST_ROW_LIFT_POSITION + (row - 1) * LIFT_INCREMENT, MAXIMUM_POSITION);

        // Return the result.
        return position;

    }

    // Determine whether this is active.
    public boolean isActive() {

        // Return indicating if this is active.
        return state != IDLE;

    }

    // Starts driving to the backdrop.
    public void startDrivingToBackdrop(double speed, double targetX) throws InterruptedException {

        // Construct a velocity constraint.
        TrajectoryVelocityConstraint velocityConstraint = new MecanumVelocityConstraint(speed, DriveConstants.TRACK_WIDTH);

        // Construct an acceleration constraint.
        TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(speed);

        // Get the drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        // Get the robot's current pose.
        Pose2d currentPose = drive.getPoseEstimate();

        // Get a target y coordinate.
        double targetY = getTargetY(leftColumn, row, redAlliance);

        // Construct a target position.
        Vector2d targetPosition = new Vector2d(targetX, targetY);

        // Construct a target pose.
        Pose2d targetPose = new Pose2d(targetPosition, Math.toRadians(180));

        // Construct a trajectory sequence.
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(currentPose)
                .setConstraints(velocityConstraint, accelerationConstraint)
                .lineToLinearHeading(targetPose)
                .build();

        // Execute the trajectory sequence.
        drive.followTrajectorySequenceAsync(sequence);

    }

    // Gets the state.
    public State getState() {

        // Return the state.
        return state;

    }

}
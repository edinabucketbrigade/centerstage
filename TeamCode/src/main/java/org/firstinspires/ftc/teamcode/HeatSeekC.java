package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HeatSeekC.State.IDLE;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.STEP_A;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.STEP_B;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.STEP_C;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.STEP_D;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.STEP_E;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.STEP_F;
import static org.firstinspires.ftc.teamcode.HeatSeekC.State.STEP_G;
import static org.firstinspires.ftc.teamcode.Lift.MAXIMUM_POSITION;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.MAXIMUM_ROW;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.MINIMUM_COLUMN;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.MINIMUM_ROW;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.getMaximumColumn;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.isEven;

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

    enum State { IDLE, STEP_A, STEP_B, STEP_C, STEP_D, STEP_E, STEP_F, STEP_G, STEP_H, STEP_I, STEP_J, STEP_K, STEP_L, STEP_M, STEP_N }

    public static double TARGET_X = 40;
    public static double TILE_SIZE = 24;
    public static double TARGET_Y_OFFSET = 7;
    public static int FIRST_ROW_LIFT_POSITION = 300;
    public static int LIFT_INCREMENT = 750;
    public static double PIXEL_WIDTH = 3;
    public static double TARGET_RED_Y = -TILE_SIZE - TARGET_Y_OFFSET;
    public static double TARGET_BLUE_Y = 2 * TILE_SIZE - TARGET_Y_OFFSET;
    public static double VELOCITY_PERCENTAGE = 0.7;
    public static double MAXIMUM_VELOCITY = DriveConstants.MAX_VEL * VELOCITY_PERCENTAGE;
    public static double MAXIMUM_ACCELERATION = DriveConstants.MAX_ACCEL * VELOCITY_PERCENTAGE;

    private RobotHardwareC robotHardware;
    private State state = IDLE;
    private ElapsedTime timer = new ElapsedTime();
    private int leftColumn;
    private int row;
    private boolean redAlliance;

    public HeatSeekC(RobotHardwareC robotHardware) {
        this.robotHardware = robotHardware;
    }

    public void start(int leftColumn, int row, boolean redAlliance) {
        if(state != IDLE) {
            return;
        }

        this.leftColumn = leftColumn;
        this.row = row;
        this.redAlliance = redAlliance;
        setState(STEP_A);
    }

    // Stops heat seeking.
    public void stop() {

        // Stop heat seeking.
        setState(IDLE);

        // TODO: Return the robot to a neutral position

        // TODO: Cancel the trajectory sequence.  See:
        //
        // Automatic Driving in Teleo
        // https://learnroadrunner.com/advanced.html#interrupting-a-live-trajectory
        //
        // Update cancelable demos
        // https://github.com/NoahBres/road-runner-quickstart/tree/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced

    }

    public void update() throws InterruptedException {

        // Get the drive interface.
        SampleMecanumDrive drive = robotHardware.getDrive();

        switch (state) {
            case STEP_A:

                // Get the robot's current pose.
                Pose2d currentPose = drive.getPoseEstimate();

                // Get a target y coordinate.
                double targetY = getTargetY(leftColumn, row, redAlliance);

                // Construct a target position.
                Vector2d targetPosition = new Vector2d(TARGET_X, targetY);

                // Construct a target pose.
                Pose2d targetPose = new Pose2d(targetPosition, Math.toRadians(180));

                // Construct a velocity constraint.
                TrajectoryVelocityConstraint velocityConstraint = new MecanumVelocityConstraint(MAXIMUM_VELOCITY, DriveConstants.TRACK_WIDTH);

                // Construct an acceleration constraint.
                TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(MAXIMUM_ACCELERATION);

                // Construct a trajectory sequence.
                TrajectorySequence sequence = drive.trajectorySequenceBuilder(currentPose)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineToLinearHeading(targetPose)
                        .build();

                // Execute the trajectory sequence.
                drive.followTrajectorySequenceAsync(sequence);

                // Advance to the next step.
                setState(STEP_B);

                break;

            case STEP_B:

                // If the robot is driving...
                if (drive.isBusy()) {

                    // Exit the method.
                    return;

                }

                // Close the claw so it does not catch when raising the arm.
                robotHardware.closeClaw();

                // Advance to the next step.
                setState(STEP_C);

                break;

            case STEP_C:

                // If we are waiting...
                if (timer.milliseconds() < 1000) {

                    // Exit the method.
                    return;

                }

                // Get a lift position.
                int liftPosition = getTargetLiftPosition(row);

                // Raise the lift.
                //robotHardware.raiseLift(liftPosition);

                // Raise the wrist.
                robotHardware.raiseWrist();

                // Raise the arm.
                robotHardware.raiseArm();

                // Advance to the next step.
                setState(STEP_D);

                break;

            case STEP_D:

                // If the arm is not up...
                if (!robotHardware.isArmUp()) {

                    // Exit the method.
                    return;

                }

                // Advance to the next step.
                setState(STEP_E);

                break;

            case STEP_E:

                // If we are waiting...
                if (timer.milliseconds() < 1000) {

                    // Exit the method.
                    return;

                }

                // Open the claw to drop the pixels.
                robotHardware.openClaw();

                // Advance to the next step.
                setState(STEP_F);

                break;

            case STEP_F:

                // If we are waiting...
                if (timer.milliseconds() < 1000) {

                    // Exit the method.
                    return;

                }

                // Close the claw.
                robotHardware.closeClaw();

                // Lower the arm.
                robotHardware.lowerArm();

                // Advance to the next step.
                setState(STEP_G);

                break;

            case STEP_G:

                // If the arm is not down...
                if (!robotHardware.isArmDown()) {

                    // Exit the method.
                    return;

                }

                // Lower the wrist.
                robotHardware.lowerWrist();

                // Open the claw.
                robotHardware.openClaw();

                // Advance to the next step.
                setState(IDLE);

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

}

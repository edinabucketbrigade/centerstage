package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HeatSeekB.State.IDLE;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_A;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_B;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_C;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_D;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_E;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_F;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_G;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_H;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_I;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_J;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_K;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_L;
import static org.firstinspires.ftc.teamcode.HeatSeekB.State.STEP_M;
import static org.firstinspires.ftc.teamcode.RobotHardwareB.MAXIMUM_ROW;
import static org.firstinspires.ftc.teamcode.RobotHardwareB.MINIMUM_COLUMN;
import static org.firstinspires.ftc.teamcode.RobotHardwareB.MINIMUM_ROW;
import static org.firstinspires.ftc.teamcode.RobotHardwareB.getMaximumColumn;
import static org.firstinspires.ftc.teamcode.RobotHardwareB.isEven;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
public class HeatSeekB {

    enum State { IDLE, STEP_A, STEP_B, STEP_C, STEP_D, STEP_E, STEP_F, STEP_G, STEP_H, STEP_I, STEP_J, STEP_K, STEP_L, STEP_M }

    public static double TARGET_X = 47;
    public static double TILE_SIZE = 24;
    public static double TARGET_Y_OFFSET = 6;
    public static int LIFT_POSITION = 2000;
    public static int LIFT_INCREMENT = 300;
    public static double PIXEL_WIDTH = 3;
    public static double TARGET_RED_Y = -TILE_SIZE - TARGET_Y_OFFSET;
    public static double TARGET_BLUE_Y = 2 * TILE_SIZE - TARGET_Y_OFFSET;

    private RobotHardwareB robotHardware;
    private State state = IDLE;
    private ElapsedTime timer = new ElapsedTime();
    private int leftColumn;
    private int row;
    private boolean redAlliance;

    public HeatSeekB(RobotHardwareB robotHardware) {
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
                //Pose2d offset = new Pose2d(0, 0.1);
                //Pose2d targetPose = currentPose.minus(offset);

                // Construct a trajectory sequence.
                TrajectorySequence sequence = drive.trajectorySequenceBuilder(currentPose)
                        .lineToLinearHeading(targetPose)
                        .build();

                // Execute the trajectory sequence.
                drive.followTrajectorySequenceAsync(sequence);

                robotHardware.offsetIntake();
                robotHardware.closeClaw();

                setState(STEP_B);

                break;

            case STEP_B:

                if (timer.milliseconds() < 500) {
                    return;
                }

                robotHardware.raiseClaw();

                setState(STEP_C);

                break;

            case STEP_C:

                if (timer.milliseconds() < 1000) {
                    return;
                }

                robotHardware.setTraversalArmPosition();

                setState(STEP_D);

                break;

            case STEP_D:

                if (timer.milliseconds() < 1000) {
                    return;
                }

                robotHardware.setPickupArmPosition();

                setState(STEP_E);

                break;

            case STEP_E:

                if (timer.milliseconds() < 1000) {
                    return;
                }

                robotHardware.openClaw();

                setState(STEP_F);

                break;

            case STEP_F:

                if (timer.milliseconds() < 500) {
                    return;
                }

                robotHardware.openGrips();

                setState(STEP_G);

                break;

            case STEP_G:

                if (timer.milliseconds() < 500) {
                    return;
                }

                // Get a lift position.
                int liftPosition = getTargetLiftPosition(row);

                robotHardware.raiseLift(liftPosition);

                setState(STEP_H);

                break;

            case STEP_H:

                if (timer.milliseconds() < 2000) {
                    return;
                }

                robotHardware.setBackdropArmPosition();

                robotHardware.closeClaw();

                setState(STEP_I);

                break;

            case STEP_I:

                if (drive.isBusy()) {
                    return;
                }

                setState(STEP_J);

                break;

            case STEP_J:

                if (timer.milliseconds() < 1000) {
                    return;
                }

                robotHardware.closeGrips();

                robotHardware.lowerClaw();

                setState(STEP_K);

                break;

            case STEP_K:

                if (timer.milliseconds() < 1000) {
                    return;
                }

                robotHardware.setNeutralArmPosition();

                robotHardware.pointIntakeDown();

                setState(STEP_L);

                break;

            case STEP_L:

                if (timer.milliseconds() < 1000) {
                    return;
                }

                robotHardware.lowerLift();

                robotHardware.openClaw();

                setState(STEP_M);

                break;

            case STEP_M:

                if (timer.milliseconds() < 1000) {
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
    public double getTargetY(int leftColumn, int row, boolean redAlliance) throws InterruptedException {

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
    private static int getTargetLiftPosition(int row) {

        // Get a heat seek lift position.
        int position =  LIFT_POSITION + (row - 1) * LIFT_INCREMENT;

        // Return the result.
        return position;

    }

    // Determine whether this is active.
    public boolean isActive() {

        // Return indicating if this is active.
        return state != IDLE;

    }

}

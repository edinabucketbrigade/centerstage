package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Hang.State.CLOSE_CLAW;
import static org.firstinspires.ftc.teamcode.Hang.State.IDLE;
import static org.firstinspires.ftc.teamcode.Hang.State.LOWER_ROBOT;
import static org.firstinspires.ftc.teamcode.Hang.State.RAISE_ARM_AND_LIFT;
import static org.firstinspires.ftc.teamcode.Hang.State.RAISE_ROBOT;
import static org.firstinspires.ftc.teamcode.Hang.State.RAISE_WRIST;
import static org.firstinspires.ftc.teamcode.Hang.State.WAIT_FOR_USER;
import static org.firstinspires.ftc.teamcode.TeleOpT.HANG_POSITION;

import static bucketbrigade.casperlibrary.RobotRoutes.MAXIMUM_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Hang {
    enum State {IDLE, CLOSE_CLAW, RAISE_WRIST, RAISE_ARM_AND_LIFT, RAISE_ROBOT, WAIT_FOR_USER, LOWER_ROBOT}

    public static int LOWER_ROBOT_SPEED = 50;

    private RobotHardwareC robotHardware;
    private State state = IDLE;
    private ElapsedTime timer = new ElapsedTime();

    public Hang(RobotHardwareC robotHardware) {
        this.robotHardware = robotHardware;
    }

    // Starts hanging.
    public void start() {

        // If we are already hanging...
        if (state != IDLE) {

            // Exit the method.
            return;

        }

        // Start raising the lift.
        setState(CLOSE_CLAW);

    }

    // Updates this.
    public void update() throws InterruptedException {

        // Get the op mode.
        OpMode opMode = robotHardware.getOpMode();

        // Get the gamepad.
        Gamepad gamepad = opMode.gamepad1;

        // Switch based on the state.
        switch (state) {

            case CLOSE_CLAW:

                robotHardware.closeClaw();

                setState(RAISE_WRIST);

                break;

            case RAISE_WRIST:

                // If we are waiting...
                if (timer.milliseconds() < 500) {

                    // Exit the method.
                    return;

                }

                robotHardware.setWristBackdrop();

                setState(RAISE_ARM_AND_LIFT);

                break;

            case RAISE_ARM_AND_LIFT:

                // If we are waiting...
                if (timer.milliseconds() < 500) {

                    // Exit the method.
                    return;

                }

                // Raise the arm
                robotHardware.raiseArm();

                // Raise the lift to its maximum height.
                robotHardware.setLiftPosition(MAXIMUM_POSITION);

                // Advance to the next step.
                setState(RAISE_ROBOT);

                break;

            case RAISE_ROBOT:

                // If the lift is not at its maximum position...
                if (!robotHardware.isLiftInPosition(MAXIMUM_POSITION)) {

                    // Exit the method.
                    return;

                }

                // If the robot driver pressed dpad up...
                if(gamepad.dpad_up) {

                    // Lower the lift to the hang position.
                    robotHardware.setLiftPosition(HANG_POSITION);

                    // Advance to the next step.
                    setState(WAIT_FOR_USER);

                }

                break;

            case WAIT_FOR_USER:

                // If the lift is not at the hang position...
                if (!robotHardware.isLiftInPosition(HANG_POSITION)) {

                    // Exit the method.
                    return;

                }

                // If the robot driver pressed dpad up...
                if(gamepad.dpad_up) {

                    // Advance to the next step.
                    setState(LOWER_ROBOT);

                }

                break;

            case LOWER_ROBOT:

                // Determine whether the robot is down (i.e. the lift is fully extended).
                boolean isRobotDown = robotHardware.isLiftInPosition(MAXIMUM_POSITION);

                // If the robot is down...
                if(isRobotDown) {

                    // Advance to the idle state.
                    setState(IDLE);

                }

                // Get the current lift position.
                int currentLiftPosition = robotHardware.getLiftPosition();

                // Raise the lift slowly to lower the robot to the ground.
                robotHardware.setLiftPosition(currentLiftPosition + LOWER_ROBOT_SPEED);

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

    // Determine whether this is active.
    public boolean isActive() {

        // Return indicating if this is active.
        return state != IDLE;

    }

    // Stops hanging.
    public void stop() {

        // Stops hanging.
        setState(IDLE);

    }

    // Gets the state.
    public State getState() {

        // Return the state.
        return state;

    }

}
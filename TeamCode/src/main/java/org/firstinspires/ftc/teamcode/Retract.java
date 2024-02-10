package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Lift.DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.Retract.State.IDLE;
import static org.firstinspires.ftc.teamcode.Retract.State.LOWER_LIFT_AND_ARM;
import static org.firstinspires.ftc.teamcode.Retract.State.LOWER_WRIST_AND_OPEN_CLAW;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Retract {
    enum State {IDLE, LOWER_LIFT_AND_ARM, LOWER_WRIST_AND_OPEN_CLAW}

    private RobotHardwareC robotHardware;
    private State state = IDLE;
    private ElapsedTime timer = new ElapsedTime();

    public Retract(RobotHardwareC robotHardware) {
        this.robotHardware = robotHardware;
    }

    // Starts retracting.
    public void start() {

        // If we are already retracting...
        if (state != IDLE) {

            // Exit the method.
            return;

        }

        // Start retracting.
        setState(LOWER_LIFT_AND_ARM);

    }

    // Updates this.
    public void update() throws InterruptedException {

        // Switch based on the state.
        switch (state) {

            case LOWER_LIFT_AND_ARM:

                // Lower the lift.
                robotHardware.setLiftPosition(DOWN_POSITION);

                // Lower the arm.
                robotHardware.lowerArm();

                // Advance to the next step.
                setState(LOWER_WRIST_AND_OPEN_CLAW);

                break;

            case LOWER_WRIST_AND_OPEN_CLAW:

                // If we are waiting...
                if (!robotHardware.isArmDown() || !robotHardware.isLiftInPosition(DOWN_POSITION)) {

                    // Exit the method.
                    return;

                }

                // Move the wrist to the ground position.
                robotHardware.setWristGround();

                // Open the claw.
                robotHardware.openClawFully();

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

    // Determine whether this is active.
    public boolean isActive() {

        // Return indicating if this is active.
        return state != IDLE;

    }

    // Gets the state.
    public State getState() {

        // Return the state.
        return state;

    }

}
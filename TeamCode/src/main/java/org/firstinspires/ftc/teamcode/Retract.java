package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Retract.State.IDLE;
import static org.firstinspires.ftc.teamcode.Retract.State.STEP_A;
import static org.firstinspires.ftc.teamcode.Retract.State.STEP_B;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Retract {
    enum State {IDLE, STEP_A, STEP_B, STEP_C, STEP_D, STEP_E, STEP_F, STEP_G, STEP_H, STEP_I, STEP_J, STEP_K, STEP_L, STEP_M, STEP_N}

    private RobotHardwareC robotHardware;
    private State state = IDLE;
    private ElapsedTime timer = new ElapsedTime();

    public Retract(RobotHardwareC robotHardware) {
        this.robotHardware = robotHardware;
    }

    public void start() {
        if (state != IDLE) {
            return;
        }

        setState(STEP_A);
    }

    public void update() throws InterruptedException {
        switch (state) {
            case STEP_A:

                // Lower the lift.
                robotHardware.lowerLift();

                // Lower the arm.
                robotHardware.lowerArm();

                // Advance to the next step.
                setState(STEP_B);

                break;

            case STEP_B:

                // If we are waiting...
                if (!robotHardware.isArmDown() && !robotHardware.isLiftDown()) {

                    // Exit the method.
                    return;

                }

                // Lower the wrist.
                robotHardware.lowerWrist();

                // Open the claw.
                robotHardware.openClawFully();

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
}

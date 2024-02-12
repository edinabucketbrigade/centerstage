package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Lift.DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.MAXIMUM_POSITION;
import static org.firstinspires.ftc.teamcode.Place.State.CLOSE_CLAW_AND_RAISE_WRIST;
import static org.firstinspires.ftc.teamcode.Place.State.IDLE;
import static org.firstinspires.ftc.teamcode.Place.State.LOWER_ARM;
import static org.firstinspires.ftc.teamcode.Place.State.LOWER_LIFT;
import static org.firstinspires.ftc.teamcode.Place.State.LOWER_WRIST_AND_OPEN_CLAW;
import static org.firstinspires.ftc.teamcode.Place.State.RAISE_ARM_AND_LIFT;
import static org.firstinspires.ftc.teamcode.Place.State.WAIT_FOR_DROP;
import static org.firstinspires.ftc.teamcode.TeleOpT.TRIGGER_THRESHOLD;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Place {
    enum State {IDLE, CLOSE_CLAW_AND_RAISE_WRIST, RAISE_WRIST, RAISE_ARM_AND_LIFT, WAIT_FOR_DROP, LOWER_ARM, LOWER_LIFT, LOWER_WRIST_AND_OPEN_CLAW }

    public static int LIFT_INCREMENT = 70;
    public static double LIFT_RAMP_MILLISECONDS = 500;

    private RobotHardwareC robotHardware;
    private State state = IDLE;
    private ElapsedTime timer = new ElapsedTime();
    private int liftPosition = DOWN_POSITION;
    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();
    private Ramp raiseLiftRamp = new Ramp(LIFT_RAMP_MILLISECONDS);
    private Ramp lowerLiftRamp = new Ramp(LIFT_RAMP_MILLISECONDS);

    public Place(RobotHardwareC robotHardware) throws InterruptedException {
        this.robotHardware = robotHardware;
    }

    // Starts placing.
    public void start() {

        // If we are already placing...
        if (state != IDLE) {

            // Exit the method.
            return;

        }

        // Start the state machine.
        setState(CLOSE_CLAW_AND_RAISE_WRIST);

    }

    // Updates this.
    public void update() throws InterruptedException {

        // Get the op mode.
        OpMode opMode = robotHardware.getOpMode();

        // Update the gamepads.
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(opMode.gamepad1);

        // Switch based on the state.
        switch (state) {

            case CLOSE_CLAW_AND_RAISE_WRIST:

                robotHardware.closeClaw();

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

                // Raise the lift.
                robotHardware.setLiftPosition(liftPosition);

                // Advance to the next step.
                setState(WAIT_FOR_DROP);

                break;

            case WAIT_FOR_DROP:

                // If the claws have dropped pixels...
                if(robotHardware.isClawOpen()) {

                    // Advance to the next step.
                    setState(LOWER_ARM);

                }

                if (currentGamepad.left_trigger > TRIGGER_THRESHOLD) {
                    int liftIncrement = (int)Math.round(lowerLiftRamp.apply(LIFT_INCREMENT));
                    liftPosition = Math.max(liftPosition - liftIncrement, DOWN_POSITION);
                    robotHardware.setLiftPosition(liftPosition);
                }
                else {
                    lowerLiftRamp.reset();
                }

                if (currentGamepad.right_trigger > TRIGGER_THRESHOLD) {
                    int liftIncrement = (int)Math.round(raiseLiftRamp.apply(LIFT_INCREMENT));
                    liftPosition = Math.min(liftPosition + liftIncrement, MAXIMUM_POSITION);
                    robotHardware.setLiftPosition(liftPosition);
                }
                else {
                    raiseLiftRamp.reset();
                }

                // If the driver pressed x...
                if(currentGamepad.x && !previousGamepad.x) {

                    // Toggle the right claw.
                    robotHardware.toggleRightClaw(false);

                }

                // If the driver pressed b...
                if(currentGamepad.b && !previousGamepad.b) {

                    // Toggle the left claw.
                    robotHardware.toggleLeftClaw(false);

                }

                // If the driver pressed a...
                if(currentGamepad.a && !previousGamepad.a) {

                    // Close the claws.
                    robotHardware.closeClaw();

                }

                // If the driver pressed y...
                if(currentGamepad.y && !previousGamepad.y) {

                    // Open the claws.
                    robotHardware.openClaw(false);

                }

                break;

            case LOWER_ARM:

                if (timer.milliseconds() < 500) {
                    return;
                }

                // Lower the arm.
                robotHardware.lowerArm();

                // Advance to the next step.
                setState(LOWER_LIFT);

                break;

            case LOWER_LIFT:

                if (timer.milliseconds() < 500) {
                    return;
                }

                // Lower the lift.
                robotHardware.setLiftPosition(DOWN_POSITION);

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
                robotHardware.openClaw(true);

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
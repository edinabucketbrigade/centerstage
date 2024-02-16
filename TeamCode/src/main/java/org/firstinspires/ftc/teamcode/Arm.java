package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ArmState.DOWN;
import static org.firstinspires.ftc.teamcode.ArmState.LOWERING;
import static org.firstinspires.ftc.teamcode.ArmState.RAISING;
import static org.firstinspires.ftc.teamcode.ArmState.UP;
import static org.firstinspires.ftc.teamcode.RobotHardwareC.areEqual;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {

    public static double LOWER_GAIN = 0.0008;
    public static double MAXIMUM_LOWER_POWER = 0.5;
    public static double MAXIMUM_RAISE_POWER = 0.8;
    public static double RAISE_EXPONENT = 2;
    public static double RAISE_GAIN = 0.0000018;
    public static int POSITION_RANGE = 750;
    public static int THRESHOLD = 50;

    private RobotHardwareC robotHardware;
    private DcMotor armMotor;
    private TouchSensor upTouch;
    private TouchSensor downTouch;
    public ArmState state = DOWN;
    private int targetPosition;

    // Initializes this.
    public Arm(RobotHardwareC robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get the hardware.
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        upTouch = hardwareMap.get(TouchSensor.class, "arm_up_touch");
        downTouch = hardwareMap.get(TouchSensor.class, "arm_down_touch");

        // Initialize the hardware.
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Update this.
    public void update() {

        // If the robot is automatically driving and we are not moving the arm...
        if(robotHardware.isAutomaticallyDriving() && state != LOWERING && state != RAISING) {

            // Exit the method.
            return;

        }

        // Get the arm's current position.
        int currentPosition = armMotor.getCurrentPosition();

        // If we are lowering the arm...
        if (state == LOWERING) {

            // Determine whether the down touch sensor is pressed.
            boolean isDownPressed = downTouch.isPressed();

            // If the arm is down...
            if(isDownPressed || areEqual(currentPosition, targetPosition, THRESHOLD)) {

                // Stop the arm motor.
                armMotor.setPower(0);

                // If the down touch sensor is pressed...
                if(isDownPressed) {

                    // Reset the arm motor.
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }

                // Remember that the arm is down.
                state = DOWN;

            }

            // Otherwise (if the arm is not down)...
            else {

                // Lower the arm.
                double positionError = Math.abs(targetPosition - currentPosition);
                double armPower = Math.min(positionError * LOWER_GAIN, MAXIMUM_LOWER_POWER);
                armMotor.setPower(-armPower);

            }

        }

        // Otherwise, if we are raising the arm...
        else if(state == RAISING) {

            // Determine whether the up touch sensor is pressed.
            boolean isUpPressed = upTouch.isPressed();

            // If the arm is up...
            if(isUpPressed || areEqual(currentPosition, targetPosition, THRESHOLD)) {

                // Stop the arm motor.
                armMotor.setPower(0);

                // Remember that the arm is up.
                state = UP;

            }

            // Otherwise (if the arm is not up)...
            else {

                // Raise the arm.
                double positionError = Math.abs(targetPosition - currentPosition);
                double armPower = Math.min(Math.pow(positionError, RAISE_EXPONENT) * RAISE_GAIN, MAXIMUM_RAISE_POWER);
                armMotor.setPower(armPower);

            }

        }

        // Otherwise (if we are not moving the arm)...
        else {

            // Stop the arm motor.
            armMotor.setPower(0);

        }

        // If the robot is automatically driving...
        if(robotHardware.isAutomaticallyDriving()) {

            // Exit the method.
            return;

        }

        // Get the arm's power.
        double power = armMotor.getPower();

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Add arm information to the telemetry.
        telemetry.addData("Arm", "State = %s, Position = %d, Power = %.2f", state, currentPosition, power);

    }

    // Raises the arm.
    public void raise() {

        // Get the arm's current position.
        int currentPosition = armMotor.getCurrentPosition();

        // Compute a target position.
        targetPosition = currentPosition + POSITION_RANGE;

        // Raise the arm.
        state = RAISING;

    }

    // Lowers the arm.
    public void lower() {

        // Get the arm's current position.
        int currentPosition = armMotor.getCurrentPosition();

        // Compute a target position.
        targetPosition = currentPosition - POSITION_RANGE;

        // Lower the arm.
        state = LOWERING;

    }

    // Waits for the user to lower the arm.
    public void waitForDown() {

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // While the arm is up...
        while (!opMode.isStopRequested() && !downTouch.isPressed()) {

            // Instruct the user to lower the arm.
            log("Please manually lower the arm...");

        }

        // If stop is requested...
        if(opMode.isStopRequested()) {

            // Exit the method.
            return;

        }

        // Notify the user that the arm is down.
        log("Arm is down");

        // Reset the arm.
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Logs a message.
    private void log(String message) {

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Show the message.
        Utilities.log(message, telemetry);

    }

    public boolean isDown() {
        return state == DOWN;
    }

    public boolean isUp() {
        return state == UP;
    }

}
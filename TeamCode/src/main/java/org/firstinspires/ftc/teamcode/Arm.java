package org.firstinspires.ftc.teamcode;

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

    public static int DOWN_POSITION = 0;
    public static double LOWER_GAIN = 0.0008;
    public static double MAXIMUM_LOWER_POWER = 0.5;
    public static double MAXIMUM_RAISE_POWER = 0.3;
    public static double RAISE_EXPONENT = 2;
    public static double RAISE_GAIN = 0.000001;
    public static int THRESHOLD = 50;
    public static int UP_POSITION = 750;

    private RobotHardwareC robotHardware;
    private DcMotor armMotor;
    private TouchSensor upTouch;
    private TouchSensor downTouch;
    private boolean isLowering;
    private boolean isRaising;

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
        if(robotHardware.isAutomaticallyDriving() && !isLowering && !isRaising) {
        //if(robotHardware.isAutomaticallyDriving()) {

            // Exit the method.
            return;

        }

        // Get the arm's current position.
        int currentPosition = armMotor.getCurrentPosition();

        // If we are lowering the arm...
        if (isLowering) {

            // Determine whether the arm is down.
            boolean isDown = isDownHelper(currentPosition);

            // If the arm is down...
            if(isDown) {

                // Remember that we finished lowering the arm.
                isLowering = false;

                // Stop the arm motor.
                armMotor.setPower(0);

            }

            // Otherwise (if the arm is not down)...
            else {

                // Lower the arm.
                double positionError = Math.abs(DOWN_POSITION - currentPosition);
                double armPower = Math.min(positionError * LOWER_GAIN, MAXIMUM_LOWER_POWER);
                armMotor.setPower(-armPower);

            }

        }

        // Otherwise, if we are raising the arm...
        else if(isRaising) {

            // Determine whether the arm is up.
            boolean isUp = isUpHelper(currentPosition);

            // If the arm is up...
            if(isUp) {

                // Remember that we finished raising the arm.
                isRaising = false;

                // Stop the arm motor.
                armMotor.setPower(0);

            }

            // Otherwise (if the arm is not up)...
            else {

                // Raise the arm.
                double positionError = Math.abs(UP_POSITION - currentPosition);
                double armPower = Math.min(Math.pow(positionError, RAISE_EXPONENT) * RAISE_GAIN, MAXIMUM_RAISE_POWER);
                armMotor.setPower(armPower);

            }

        }

        // Otherwise (if we are not moving the arm)...
        else {

            // Stop the arm motor.
            armMotor.setPower(0);

            // Determine whether the down touch sensor is pressed.
            boolean isDownPressed = downTouch.isPressed();

            // If the down touch sensor is pressed...
            if(isDownPressed) {

                // Reset the arm motor.
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }

        }

        // If the robot is automatically driving...
        if(robotHardware.isAutomaticallyDriving()) {

            // Exit the method.
            return;

        }

        // Get the arm's power.
        double power = armMotor.getPower();

        // Determine whether the arm is up.
        boolean isUp = isUpHelper(currentPosition);

        // Determine whether the arm is down.
        boolean isDown = isDownHelper(currentPosition);

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Add arm information to the telemetry.
        telemetry.addData("Arm", "Down = %b, Up = %b, Lowering = %b, Raising = %b, Position = %d, Power = %.2f", isDown, isUp, isLowering, isRaising, currentPosition, power);

    }

    // Raises the arm.
    public void raise() {

        // Raise the arm.
        isLowering = false;
        isRaising = true;

    }

    // Lowers the arm.
    public void lower() {

        // Lower the arm.
        isLowering = true;
        isRaising = false;

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
        int currentPosition = armMotor.getCurrentPosition();
        return isInPosition(currentPosition, DOWN_POSITION);
    }

    private boolean isDownHelper(int currentPosition) {
        return isInPosition(currentPosition, DOWN_POSITION);
    }

    public boolean isUp() {
        int currentPosition = armMotor.getCurrentPosition();
        return isInPosition(currentPosition, UP_POSITION);
    }

    private boolean isUpHelper(int currentPosition) {
        return isInPosition(currentPosition, UP_POSITION);
    }

    private boolean isInPosition(int currentPosition, int targetPosition) {
        int difference = Math.abs(currentPosition - targetPosition);
        return difference < THRESHOLD;
    }

}
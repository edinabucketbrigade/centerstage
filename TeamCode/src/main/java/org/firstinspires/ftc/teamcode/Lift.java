package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {

    public static int DOWN_POSITION = 0;
    public static double LOWER_POWER = 0.6;
    public static int MAXIMUM_POSITION = 1650;
    public static double RAISE_POWER = 1;
    public static int THRESHOLD = 50;

    private RobotHardwareC robotHardware;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private TouchSensor touch;
    private boolean isLowering;
    private int targetPosition;

    // Initializes this.
    public Lift(RobotHardwareC robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get hardware.
        leftMotor = hardwareMap.get(DcMotor.class,"left_lift_motor");
        rightMotor = hardwareMap.get(DcMotor.class,"right_lift_motor");
        touch = hardwareMap.get(TouchSensor.class, "lift_touch");

        // Initialize hardware.
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    // Update this.
    public void update() {

        // If the robot is automatically driving...
        if(robotHardware.isAutomaticallyDriving()) {

            // Exit the method.
            return;

        }

        // Determine whether the lift is down.
        boolean isDownPressed = touch.isPressed();

        // Get the lift's position.
        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();

        // If we finished lowering the lift...
        if (isLowering && isDownPressed) {

            // Reset the lift.
            reset();

            // Remember that we finished lowering the lift.
            isLowering = false;

        }

        // Get the lift's power.
        double leftPower = leftMotor.getPower();
        double rightPower = rightMotor.getPower();

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Add lift information to the telemetry.
        telemetry.addData("Lift", "Lowering = %b, Position = %d/%d, Power = %.2f/%.2f", isLowering, leftPosition, rightPosition, leftPower, rightPower);

    }

    // Resets the lift.
    private void reset() {

        // Reset the lift motors.
        resetMotor(leftMotor);
        resetMotor(rightMotor);

    }

    // Resets a lift motor.
    private static void resetMotor(DcMotor liftMotor) {

        // Reset the lift motor.
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Raises the lift.
    public void raise(int position) {

        // Set target position.
        targetPosition = position;

        // Raise the lift.
        setPosition(leftMotor, position, RAISE_POWER);
        setPosition(rightMotor, position, RAISE_POWER);
        isLowering = false;

    }

    // Lowers the lift.
    public void lower() {

        // Lower the lift.
        setPosition(leftMotor, DOWN_POSITION, LOWER_POWER);
        setPosition(rightMotor, DOWN_POSITION, LOWER_POWER);
        isLowering = true;

    }

    // Sets the lift position.
    private void setPosition(DcMotor liftMotor, int position, double power) {

        // Set the lift position.
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);

    }

    // Waits for the user to lower the lift.
    public void waitForDown() {

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get gamepad 1.
        Gamepad gamepad1 = opMode.gamepad1;

        // While the lift is up...
        while (!opMode.isStopRequested() && !touch.isPressed()) {

            // Instruct the user to lower the lift.
            log("Hold gamepad 1 back to lower lift...");

            // If the user is pressing back...
            if (gamepad1.back) {

                // Lower the lift.
                leftMotor.setPower(-LOWER_POWER);
                rightMotor.setPower(-LOWER_POWER);

            }

            // Otherwise (if the user is not pressing back)...
            else {

                // Stop the lift.
                leftMotor.setPower(0);
                rightMotor.setPower(0);

            }

        }

        // If stop is requested...
        if(opMode.isStopRequested()) {

            // Exit the method.
            return;

        }

        // Notify the user that the lift is down.
        log("Lift is down");

        // Reset the lift.
        reset();

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
        return isInPosition(DOWN_POSITION);
    }

    public boolean isUp() {
        return isInPosition(targetPosition);
    }

    private boolean isInPosition(int targetPosition) {
        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();
        int leftDifference = Math.abs(leftPosition - targetPosition);
        int rightDifference = Math.abs(rightPosition - targetPosition);
        if (leftDifference < THRESHOLD && rightDifference < THRESHOLD) {
            return true;
        }
        else {
            return false;
        }
    }

}

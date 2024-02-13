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
public class Lift {

    public static int DOWN_POSITION = 0;
    public static double LOWER_POWER = 0.6;
    public static double RAISE_POWER = 1;
    public static int THRESHOLD = 50;

    private RobotHardwareC robotHardware;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private TouchSensor touch;
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

        // Get the lift's position.
        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();

        // If we finished lowering the lift...
        if(targetPosition == DOWN_POSITION && touch.isPressed()) {

            // Reset the lift.
            reset();

        }

        // Get the lift's power.
        double leftPower = leftMotor.getPower();
        double rightPower = rightMotor.getPower();

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Add lift information to the telemetry.
        telemetry.addData("Lift", "Position = %d/%d, Power = %.2f/%.2f", leftPosition, rightPosition, leftPower, rightPower);

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

    // Sets the lift's position.
    public void setPosition(int targetPosition) {

        // Remember the target position.
        this.targetPosition = targetPosition;

        // Get the lift's current position.
        int currentPosition = leftMotor.getCurrentPosition();

        // Get the appropriate power.
        double power = targetPosition > currentPosition ? RAISE_POWER : LOWER_POWER;

        // Lower the lift.
        setPosition(leftMotor, targetPosition, power);
        setPosition(rightMotor, targetPosition, power);

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

        // While the lift is up...
        while (!opMode.isStopRequested() && !touch.isPressed()) {

            // Instruct the user to lower the lift.
            log("Please manually lower the lift...");

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

    public boolean isInPosition(int targetPosition) {
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

    public int getLiftPosition() {
        return leftMotor.getCurrentPosition();
    }

}

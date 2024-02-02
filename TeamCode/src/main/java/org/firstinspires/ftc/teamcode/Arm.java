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
    public static double LOWER_GAIN = 0.0006;
    public static double MAXIMUM_LOWER_POWER = 0.4;
    public static double MAXIMUM_RAISE_POWER = 1;
    public static double RAISE_GAIN = 0.0007;
    public static int UP_POSITION = 750;

    private LinearOpMode opMode;
    private DcMotor armMotor;
    private TouchSensor upTouch;
    private TouchSensor downTouch;
    private boolean isLowering;
    private boolean isRaising;

    // Initializes this.
    public Arm(LinearOpMode opMode) {

        // Remember the op mode.
        this.opMode = opMode;

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

        // Get the arm's current position.
        int currentPosition = armMotor.getCurrentPosition();

        // Get the arm's power.
        double power = armMotor.getPower();

        // Get the arm touch sensor values.
        boolean isDown = downTouch.isPressed();
        boolean isUp = upTouch.isPressed();

        // If we are lowering the arm...
        if (isLowering) {

            // If the arm is down...
            if(isDown) {

                // Reset the arm.
                reset();

                // Remember that we finished lowering the arm.
                isLowering = false;

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

            // If the arm is up...
            if(isUp) {

                // Stop the arm motor.
                armMotor.setPower(0);

                // Remember that we finished raising the arm.
                isRaising = false;

            }

            // Otherwise (if the arm is not down)...
            else {

                // Raise the arm.
                double positionError = Math.abs(UP_POSITION - currentPosition);
                double armPower = Math.min(positionError * RAISE_GAIN, MAXIMUM_RAISE_POWER);
                armMotor.setPower(armPower);

            }

        }

        // Otherwise (if we are not moving the arm)...
        else {

            // Stop the arm motor.
            armMotor.setPower(0);

        }

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

        // While the arm is up...
        while (!opMode.isStopRequested() && !downTouch.isPressed()) {

            // Instruct the user to lower the arm.
            log("Please lower the arm...");

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

        // If the op mode is missing...
        if (opMode == null) {

            // Exit the method.
            return;

        }

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Show the message.
        Utilities.log(message, telemetry);

    }

    // Resets the arm.
    private void reset() {

        // Reset the arm motor.
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
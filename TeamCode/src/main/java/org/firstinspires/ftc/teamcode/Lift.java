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

    public static double RAISE_LIFT_POWER = 1;
    public static double LOWER_LIFT_POWER = 0.6;
    public static int LIFT_DOWN_POSITION = 0;
    public static int MAXIMUM_LIFT_POSITION = 1650;

    private LinearOpMode opMode;
    private DcMotor leftLiftMotor;
    private DcMotor rightLiftMotor;
    private TouchSensor liftTouch;
    private boolean isLoweringLift;

    // Initializes this.
    public Lift(LinearOpMode opMode) {

        // Remember the op mode.
        this.opMode = opMode;

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get hardware.
        leftLiftMotor = hardwareMap.get(DcMotor.class,"left_lift_motor");
        rightLiftMotor = hardwareMap.get(DcMotor.class,"right_lift_motor");
        liftTouch = hardwareMap.get(TouchSensor.class, "lift_touch");

        // Initialize hardware.
        leftLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    // Update this.
    public void update() {

        // Determine whether the lift is down.
        boolean isLiftDown = liftTouch.isPressed();

        // Get the lift's position.
        int leftLiftPosition = leftLiftMotor.getCurrentPosition();
        int rightLiftPosition = rightLiftMotor.getCurrentPosition();

        // If we finished lowering the lift...
        if (isLoweringLift && isLiftDown) {

            // Reset the lift.
            resetLift();

            // Remember that we finished lowering the lift.
            isLoweringLift = false;

        }

        // Get the lift's power.
        double leftLiftPower = leftLiftMotor.getPower();
        double rightLiftPower = rightLiftMotor.getPower();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Add lift information to the telemetry.
        telemetry.addData("Lift", "Down = %b, Lowering = %b, Position = %d/%d, Power = %.2f/%.2f", isLiftDown, isLoweringLift, leftLiftPosition, rightLiftPosition, leftLiftPower, rightLiftPower);

    }

    // Resets the lift.
    private void resetLift() {

        // Reset the lift motors.
        resetLiftMotor(leftLiftMotor);
        resetLiftMotor(rightLiftMotor);

    }

    // Resets a lift motor.
    private static void resetLiftMotor(DcMotor liftMotor) {

        // Reset the lift motor.
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Raises the lift.
    public void raise(int position) {

        // Raise the lift.
        setLiftPosition(leftLiftMotor, position, RAISE_LIFT_POWER);
        setLiftPosition(rightLiftMotor, position, RAISE_LIFT_POWER);
        isLoweringLift = false;

    }

    // Lowers the lift.
    public void lower() throws InterruptedException {

        // Lower the lift.
        setLiftPosition(leftLiftMotor, LIFT_DOWN_POSITION, LOWER_LIFT_POWER);
        setLiftPosition(rightLiftMotor, LIFT_DOWN_POSITION, LOWER_LIFT_POWER);
        isLoweringLift = true;

    }

    // Sets the lift position.
    private void setLiftPosition(DcMotor liftMotor, int position, double power) {

        // Set the lift position.
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);

    }

    // Waits for the user to lower the lift.
    public void waitForDown() throws InterruptedException {

        // Get gamepad 1.
        Gamepad gamepad1 = opMode.gamepad1;

        // While the lift is up...
        while (!opMode.isStopRequested() && !liftTouch.isPressed()) {

            // Instruct the user to lower the lift.
            log("Hold gamepad 1 back to lower lift...");

            // If the user is pressing back...
            if (gamepad1.back) {

                // Lower the lift.
                leftLiftMotor.setPower(-LOWER_LIFT_POWER);
                rightLiftMotor.setPower(-LOWER_LIFT_POWER);

            }

            // Otherwise (if the user is not pressing back)...
            else {

                // Stop the lift.
                leftLiftMotor.setPower(0);
                rightLiftMotor.setPower(0);

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
        resetLift();

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

}

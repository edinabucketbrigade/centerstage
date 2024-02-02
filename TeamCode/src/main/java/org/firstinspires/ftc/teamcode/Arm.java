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

    public static double RAISE_POWER = 0.6;
    public static double LOWER_POWER = 0.2;
    public static int UP_POSITION = 750;
    public static int DOWN_POSITION = 0;

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
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Update this.
    public void update() {

        // Get the arm's position.
        int position = armMotor.getCurrentPosition();

        // Get the arm's power.
        double power = armMotor.getPower();

        // Get the arm touch sensor values.
        boolean isDown = downTouch.isPressed();
        boolean isUp = upTouch.isPressed();

        // If we finished lowering the arm...
        if (isLowering && isDown) {

            // Reset the arm.
            reset();

            // Remember that we finished lowering the arm.
            isLowering = false;

        }

        // If we finished raising the arm...
        if(isRaising && isUp) {

            // Stop the arm motor.
            armMotor.setPower(0);

            // Remember that we finished raising the arm.
            isRaising = false;

        }

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Add arm information to the telemetry.
        telemetry.addData("Arm", "Down = %b, Up = %b, Lowering = %b, Raising = %b, Position = %d, Power = %.2f", isDown, isUp, isLowering, isRaising, position, power);

    }

    // Raises the arm.
    public void raise() {

        // Raise the arm.
        armMotor.setTargetPosition(UP_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(RAISE_POWER);
        isLowering = false;
        isRaising = true;

    }

    // Lowers the arm.
    public void lower() {

        // Lower the arm.
        armMotor.setTargetPosition(DOWN_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(LOWER_POWER);
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
        reset();

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
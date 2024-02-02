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

    public static double RAISE_ARM_POWER = 0.6;
    public static double LOWER_ARM_POWER = 0.2;
    public static int ARM_UP_POSITION = 750;
    public static int ARM_DOWN_POSITION = 0;

    private LinearOpMode opMode;
    private DcMotor armMotor;
    private TouchSensor armUpTouch;
    private TouchSensor armDownTouch;

    // Initializes this.
    public Arm(LinearOpMode opMode) {

        // Remember the op mode.
        this.opMode = opMode;

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get the hardware.
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armUpTouch = hardwareMap.get(TouchSensor.class, "arm_up_touch");
        armDownTouch = hardwareMap.get(TouchSensor.class, "arm_down_touch");

        // Initialize the hardware.
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Update this.
    public void update() {

        // Get the arm's position.
        int armPosition = armMotor.getCurrentPosition();

        // Get the arm's power.
        double armPower = armMotor.getPower();

        // Get the arm touch sensor values.
        boolean armDownPressed = armDownTouch.isPressed();
        boolean armUpPressed = armUpTouch.isPressed();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Add arm information to the telemetry.
        telemetry.addData("Arm", "Position = %d, Power = %.2f, Down Pressed = %b, Up Pressed = %b", armPosition, armPower, armDownPressed, armUpPressed);

    }

    // Raises the arm.
    public void raise() {

        // Raise the arm.
        armMotor.setTargetPosition(ARM_UP_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(RAISE_ARM_POWER);

    }

    // Lowers the arm.
    public void lower() {

        // Lower the arm.
        armMotor.setTargetPosition(ARM_DOWN_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(LOWER_ARM_POWER);

    }

}
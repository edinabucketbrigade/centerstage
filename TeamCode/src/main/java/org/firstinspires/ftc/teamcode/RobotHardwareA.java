package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class RobotHardwareA {

    /*
    Control Hub Portal
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - arm_motor
            Digital Devices
                7 - REV Touch Sensor - touch
        Control Hub
            Motors
                0 - GoBILDA 5201 series - right_back_drive
                1 - GoBILDA 5201 series - right_front_drive
                2 - GoBILDA 5201 series - left_front_drive
                3 - GoBILDA 5201 series - left_back_drive
            Servos
                0 - Servo - right_claw_servo
                1 - Servo - left_claw_servo
                2 - Servo - wrist_servo
    Webcam 1
    */

    public static double LEFT_CLAW_OPEN_POSITION = 0.7;
    public static double LEFT_CLAW_CLOSED_POSITION = 1;
    public static double RIGHT_CLAW_CLOSED_POSITION = 0.275;
    public static double RIGHT_CLAW_OPEN_POSITION = 0.6;
    public static double WRIST_DOWN_POSITION = 0;
    public static double WRIST_UP_POSITION = 0.72;
    public static int ARM_DOWN_POSITION = 0;
    public static int ARM_UP_POSITION = 1300;
    public static double ARM_RAISE_POWER = 0.8;
    public static double ARM_LOWER_POWER = 0.4;
    private static final String TAG = "Bucket Brigade";
    private LinearOpMode myOpMode;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor armMotor;

    private Servo wristServo;
    private Servo leftClawServo;
    private Servo rightClawServo;

    private TouchSensor touchSensor;

    private boolean leftClawIsOpen;
    private boolean rightClawIsOpen;
    private boolean wristIsUp;
    private boolean armIsUp;

    private boolean isReady;

    public RobotHardwareA (LinearOpMode opmode) {
        myOpMode = opmode;

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");

        wristServo = myOpMode.hardwareMap.get(Servo.class, "wrist_servo");
        leftClawServo = myOpMode.hardwareMap.get(Servo.class, "left_claw_servo");
        rightClawServo = myOpMode.hardwareMap.get(Servo.class, "right_claw_servo");

        touchSensor = myOpMode.hardwareMap.get(TouchSensor.class, "touch");

        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "arm_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        Telemetry telemetry = myOpMode.telemetry;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void toggleLeftClaw() {
        double leftClawPosition = leftClawIsOpen ? LEFT_CLAW_CLOSED_POSITION : LEFT_CLAW_OPEN_POSITION;
        leftClawIsOpen = !leftClawIsOpen;
        log("left claw position: " + leftClawPosition + ", left claw is open: " + leftClawIsOpen);
        leftClawServo.setPosition(leftClawPosition);
    }

    public void toggleRightClaw() {
        double rightClawPosition = rightClawIsOpen ? RIGHT_CLAW_CLOSED_POSITION : RIGHT_CLAW_OPEN_POSITION;
        rightClawIsOpen = !rightClawIsOpen;
        log("right claw position: " + rightClawPosition + ", right claw is open: " + rightClawIsOpen);
        rightClawServo.setPosition(rightClawPosition);
    }

    public void toggleWrist() {
        double wristPosition = wristIsUp ? WRIST_DOWN_POSITION : WRIST_UP_POSITION;
        wristIsUp = !wristIsUp;
        log("wrist position: " + wristPosition + ", wrist is up: " + wristIsUp);
        wristServo.setPosition(wristPosition);
    }

    public void setDrivePowerManually(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void initializeArm() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void toggleArm() {
        if (!isReady) {
            return;
        }
        int armPosition;
        if (armIsUp) {
            armPosition = ARM_DOWN_POSITION;
            armMotor.setPower(ARM_LOWER_POWER);
        } else {
            armPosition = ARM_UP_POSITION;
            armMotor.setPower(ARM_RAISE_POWER);
        }
        armIsUp = !armIsUp;
        log("arm position: " + armPosition + ", arm is up: " + armIsUp);
        armMotor.setTargetPosition(armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void update() {
        boolean isPressed = touchSensor.isPressed();
        if (isPressed && !isReady) {
            initializeArm();
            isReady = true;
        }

        Telemetry telemetry = myOpMode.telemetry;

        telemetry.addData("Status", "Running");
        telemetry.addData("Ready", isReady);
        telemetry.addData("Arm Motor Position", armMotor.getCurrentPosition());
        telemetry.addData("Arm Motor Power", armMotor.getPower());
        telemetry.addData("Left Front Motor Position", leftFrontDrive.getCurrentPosition());
        telemetry.addData("Left Front Motor Power", leftFrontDrive.getPower());
        telemetry.addData("Left Back Motor Position", leftBackDrive.getCurrentPosition());
        telemetry.addData("Left Back Motor Power", leftBackDrive.getPower());
        telemetry.addData("Right Front Motor Position", rightFrontDrive.getCurrentPosition());
        telemetry.addData("Right Front Motor Power", rightFrontDrive.getPower());
        telemetry.addData("Right Back Motor Position", rightBackDrive.getCurrentPosition());
        telemetry.addData("Right Back Motor Power", rightBackDrive.getPower());
        telemetry.addData("Touch Sensor Pressed", touchSensor.isPressed());
        telemetry.addData("Left Claw Servo Position", leftClawServo.getPosition());
        telemetry.addData("Right Claw Servo Position", rightClawServo.getPosition());
        telemetry.addData("Wrist Servo Position", wristServo.getPosition());
        telemetry.update();
    }

    public void setDrivePowerAutomatically() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -myOpMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = myOpMode.gamepad1.left_stick_x;
        double yaw = myOpMode.gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        //lift.setPower(liftPower);

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

        /*leftBackPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
        leftFrontPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
        rightBackPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
        rightFrontPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad*/

        // Send calculated power to wheels
        setDrivePowerManually(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void log(String message) {
        Log.d(TAG, message);
    }
}

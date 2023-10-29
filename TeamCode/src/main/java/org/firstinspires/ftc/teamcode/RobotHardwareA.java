package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class RobotHardwareA {
    public static double LEFT_CLAW_MINIMUM = 0.275;
    public static double LEFT_CLAW_MAXIMUM = 0.6;
    public static double RIGHT_CLAW_MINIMUM = 0.7;
    public static double RIGHT_CLAW_MAXIMUM = 1;
    public static double WRIST_MINIMUM = 0;
    public static double WRIST_MAXIMUM = 0.72;
    public static int ARM_MINIMUM = 0;
    public static int ARM_MAXIMUM = 1300;
    public static double ARM_RAISE_POWER = 0.8;
    public static double ARM_LOWER_POWER = 0.4;
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

    private double leftClawPosition;
    private double rightClawPosition;
    private double wristPosition;

    private int armPosition;

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

        leftClawPosition = LEFT_CLAW_MINIMUM;
        rightClawPosition = RIGHT_CLAW_MINIMUM;
        wristPosition = WRIST_MINIMUM;

        armPosition = ARM_MINIMUM;

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }


    public void toggleLeftClaw() {
        leftClawPosition = leftClawPosition == LEFT_CLAW_MINIMUM ? LEFT_CLAW_MAXIMUM : LEFT_CLAW_MINIMUM;
        leftClawServo.setPosition(leftClawPosition);
    }
    public void toggleRightClaw() {
        rightClawPosition = rightClawPosition == RIGHT_CLAW_MINIMUM ? RIGHT_CLAW_MAXIMUM : RIGHT_CLAW_MINIMUM;
        rightClawServo.setPosition(rightClawPosition);
    }
    public void toggleWrist() {
        wristPosition = wristPosition == WRIST_MINIMUM ? WRIST_MAXIMUM : WRIST_MINIMUM;
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
        if (armPosition == ARM_MINIMUM) {
            armPosition = ARM_MAXIMUM;
            armMotor.setPower(ARM_RAISE_POWER);
        } else {
            armPosition = ARM_MINIMUM;
            armMotor.setPower(ARM_LOWER_POWER);
        }
        armMotor.setTargetPosition(armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void update() {
        boolean isPressed = touchSensor.isPressed();
        if (isPressed && !isReady) {
            initializeArm();
            isReady = true;
        }
        myOpMode.telemetry.addData("Ready", isReady);
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

        myOpMode.telemetry.addData("Front left/right power", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        myOpMode.telemetry.addData("Back  left/right power", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }
}

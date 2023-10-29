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

    public TouchSensor touchSensor;

    private double leftClawPosition;
    private double rightClawPosition;
    private double wristPosition;

    private int armPosition;

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
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    public void initializeArm() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void toggleArm() {
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
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class RobotHardwareA {
    private LinearOpMode myOpMode = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    public DcMotor armMotor = null;

    public Servo wristServo = null;
    public Servo leftClawServo = null;
    public Servo rightClawServo = null;

    public TouchSensor touchSensor;

    public RobotHardwareA (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init()    {
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
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

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
}

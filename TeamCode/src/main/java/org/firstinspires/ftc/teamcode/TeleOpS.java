package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class TeleOpS extends LinearOpMode {
    /*
    Control Hub Portal
        Control Hub
            Motors
                0 - GoBILDA 5201 series - left_lift_motor (has encoder) **Adapter was taken for arm**
                1 - GoBILDA 5201 series - front_encoder (is encoder)
                2 - GoBILDA 5201 series - left_back_drive (encoder port returns 0 and -1)
                3 - GoBILDA 5201 series - left_front_drive (left encoder)
            Digital Devices
                6 - Digital Device - green_left_led
                7 - Digital Device - red_left_led
            Servos
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - arm_motor (has encoder) **Adapter was taken from left lift motor**
                1 - GoBILDA 5201 series - right_lift_motor (has encoder)
                2 - GoBILDA 5201 series - right_front_drive (encoder port has bent pin)
                3 - GoBILDA 5201 series - right_back_drive (right encoder)
            Digital Devices
                3 - REV Touch Sensor - lift_touch
            I2C
                1 - REV 2m Distance Sensor - arm_up_distance
                2 - REV 2m Distance Sensor - arm_down_distance
            Servos
                0
                1 - Servo - wrist_servo
                2 - Servo - right_claw_servo
                3 - Servo - left_claw_servo
       Webcam 1
    */

    public static double ARM_POWER = 0.2;
    public static int ARM_UP_POSITION = 300;
    public static int ARM_DOWN_POSITION = 0;
    public static double WRIST_SERVO_DOWN_POSITION = 0.4;
    public static double WRIST_SERVO_UP_POSITION = 0.6;
    public static double LEFT_CLAW_SERVO_CLOSED_POSITION = 0.4;
    public static double LEFT_CLAW_SERVO_OPEN_POSITION = 0.6;
    public static double RIGHT_CLAW_SERVO_CLOSED_POSITION = 0.4;
    public static double RIGHT_CLAW_SERVO_OPEN_POSITION = 0.6;

    private DcMotor armMotor;
    private DistanceSensor armUpDistance;
    private DistanceSensor armDownDistance;
    private DigitalChannel greenLeftLed;
    private DigitalChannel redLeftLed;

    private boolean wristServoUp;
    private boolean leftClawServoOpen;
    private boolean rightClawServoOpen;

    private Servo wristServo;
    private Servo rightClawServo;
    private Servo leftClawServo;

    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() {
        FtcDashboard.getInstance();

        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armUpDistance = hardwareMap.get(Rev2mDistanceSensor.class, "arm_up_distance");
        armDownDistance = hardwareMap.get(Rev2mDistanceSensor.class, "arm_down_distance");

        greenLeftLed = hardwareMap.get(DigitalChannel.class, "green_left_led");
        redLeftLed = hardwareMap.get(DigitalChannel.class, "red_left_led");

        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            telemetry.addData("Status", "Running");
            telemetry.addData("Arm Up Distance Sensor", String.format("%.01f mm", armUpDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("Arm Down Distance Sensor", String.format("%.01f mm", armDownDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("Arm Motor Position", armMotor.getCurrentPosition());
            telemetry.addData("Arm Motor Power", armMotor.getPower());
            telemetry.update();

            greenLeftLed.setMode(DigitalChannel.Mode.OUTPUT);
            redLeftLed.setMode(DigitalChannel.Mode.OUTPUT);

            if (currentGamepad.y && !previousGamepad.y) {
                //armMotor.setPower(ARM_POWER);
                armMotor.setTargetPosition(ARM_UP_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER);
            }
            else if (currentGamepad.a && !previousGamepad.a) {
                //armMotor.setPower(-ARM_POWER);
                armMotor.setTargetPosition(ARM_DOWN_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER);
            }

            if (currentGamepad.x && !previousGamepad.x) {
                greenLeftLed.setState(true);
                redLeftLed.setState(false);
            }

            if (currentGamepad.b && !previousGamepad.b) {
                greenLeftLed.setState(false);
                redLeftLed.setState(true);
            }

            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                if (wristServoUp) {
                    wristServoUp = false;
                    wristServo.setPosition(WRIST_SERVO_DOWN_POSITION);
                }
                else {
                    wristServoUp = true;
                    wristServo.setPosition(WRIST_SERVO_UP_POSITION);
                }
            }

            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                if (leftClawServoOpen) {
                    leftClawServoOpen = false;
                    leftClawServo.setPosition(LEFT_CLAW_SERVO_CLOSED_POSITION);
                }
                else {
                    leftClawServoOpen = true;
                    leftClawServo.setPosition(LEFT_CLAW_SERVO_OPEN_POSITION);
                }
            }

            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                if (rightClawServoOpen) {
                    rightClawServoOpen = false;
                    rightClawServo.setPosition(RIGHT_CLAW_SERVO_CLOSED_POSITION);
                }
                else {
                    rightClawServoOpen = true;
                    rightClawServo.setPosition(RIGHT_CLAW_SERVO_OPEN_POSITION);
                }
            }

        }

    }

}
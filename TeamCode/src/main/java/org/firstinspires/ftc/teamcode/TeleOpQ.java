package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
@Config
public class TeleOpQ extends LinearOpMode {

    public static double ELBOW_PICK_UP = 0.05;
    public static double ELBOW_BACKDROP = 0.67;
    public static double WRIST_PICK_UP = 0.75;
    public static double WRIST_BACKDROP = 0.2;
    public static double RIGHT_MAXIMUM_HAND_POSITION = 0.25;
    public static double RIGHT_MINIMUM_HAND_POSITION = 0.38;
    public static double LEFT_MAXIMUM_HAND_POSITION = 0.35;
    public static double LEFT_MINIMUM_HAND_POSITION = 0.46;

    private Servo elbowServo;
    private Servo wristServo;
    private Servo leftHandServo;
    private Servo rightHandServo2;

    @Override
    public void runOpMode() {
        FtcDashboard.getInstance();

        elbowServo = hardwareMap.get(Servo.class, "elbow_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        leftHandServo = hardwareMap.get(Servo.class, "left_hand_servo");
        rightHandServo2 = hardwareMap.get(Servo.class, "right_hand_servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            if(gamepad1.a){
                elbowServo.setPosition(ELBOW_PICK_UP);
                wristServo.setPosition(WRIST_PICK_UP);

            }
            if(gamepad1.y){
                elbowServo.setPosition(ELBOW_BACKDROP);
                wristServo.setPosition(WRIST_BACKDROP);

            }
            if(gamepad1.x){
                leftHandServo.setPosition(LEFT_MINIMUM_HAND_POSITION);
                rightHandServo2.setPosition(RIGHT_MINIMUM_HAND_POSITION);
            }
            if(gamepad1.b){
               leftHandServo.setPosition(LEFT_MAXIMUM_HAND_POSITION);
                rightHandServo2.setPosition(RIGHT_MAXIMUM_HAND_POSITION);
            }
        }

    }

}
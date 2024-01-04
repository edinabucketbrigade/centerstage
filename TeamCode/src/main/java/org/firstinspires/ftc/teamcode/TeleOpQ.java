package org.firstinspires.ftc.teamcode;

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
public class TeleOpQ extends LinearOpMode {

    public static double ELBOW_PICK_UP = 0.05;
    public static double ELBOW_BACKDROP = 0.67;
    public static double WRIST_PICK_UP = 0.75;
    public static double WRIST_BACKDROP = 0.2;
    public static double MINIMUM_HAND_POSITION = 0.5;
    public static double MAXIMUM_HAND_POSITION = 0;

    private Servo elbowServo;
    private Servo wristServo;
    private Servo handServo;

    @Override
    public void runOpMode() {

        elbowServo = hardwareMap.get(Servo.class, "elbow_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        handServo = hardwareMap.get(Servo.class, "hand_servo");

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
                handServo.setPosition(MINIMUM_HAND_POSITION);
            }
            if(gamepad1.b){
                handServo.setPosition(MAXIMUM_HAND_POSITION);
            }
        }

    }

}
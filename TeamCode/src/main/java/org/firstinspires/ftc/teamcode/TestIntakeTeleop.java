package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Config
//@Disabled
public class TestIntakeTeleop extends LinearOpMode {

    private DcMotor intake_motor;
    private Servo left_servo;
    private Servo right_servo;
    private Servo intake_servo;
    private Servo flip_servo;
    public static double INTAKE_MOTOR_FLIP_POSITION = 0.1;
    public static double INTAKE_SERVO_FLIP_POSITION = 0.572;
    public static double INTAKE_SERVO_NORMAL_POSITION = 0;

    @Override
    public void runOpMode() {
        FtcDashboard.getInstance();

        intake_motor  = hardwareMap.get(DcMotor.class, "intake_motor");
        left_servo = hardwareMap.get(Servo.class, "left_servo");
        right_servo = hardwareMap.get(Servo.class, "right_servo");
        intake_servo = hardwareMap.get(Servo.class, "intake_servo");
        flip_servo = hardwareMap.get(Servo.class, "flip_servo");

        telemetry.addData("Status", "Test Motors and Servo");
        telemetry.update();

        waitForStart();

        double motorPower = 0;
        while (opModeIsActive()) {
            motorPower = -this.gamepad1.left_stick_y;
            intake_motor.setPower(motorPower);

            telemetry.addData("Encoder Value", intake_motor.getCurrentPosition());
            telemetry.addData("Status", "Test Running");
            telemetry.update();

            if(gamepad1.dpad_left) {
                right_servo.setPosition(1);
                left_servo.setPosition(0);
            }
            if(gamepad1.dpad_right) {
                right_servo.setPosition(0);
                left_servo.setPosition(1);
                intake_servo.setPosition(INTAKE_SERVO_NORMAL_POSITION);
            }
            if (gamepad1.x) {
                right_servo.setPosition(0);
            }
            if (gamepad1.a) {
                right_servo.setPosition(1);
            }

            if (gamepad1.y){
                //move to -135 degrees
                left_servo.setPosition(1);

            } else if (gamepad1.b) {
                //move to 135 degrees
                left_servo.setPosition(0);
            }
            double intake_servo_position = intake_servo.getPosition();

            if (gamepad1.dpad_up) {
                intake_servo.setPosition(intake_servo_position + 0.001);
            }

            if (gamepad1.dpad_down) {
                intake_servo.setPosition(intake_servo_position - 0.001);
            }

            if(gamepad1.right_bumper) {
                flip_servo.setPosition(INTAKE_SERVO_FLIP_POSITION);
                sleep(1000);
                right_servo.setPosition(0);
                left_servo.setPosition(1);
                intake_servo.setPosition(INTAKE_SERVO_NORMAL_POSITION);
            }
            if(gamepad1.left_bumper) {
                intake_servo.setPosition(INTAKE_MOTOR_FLIP_POSITION);
                left_servo.setPosition(0.5);
                right_servo.setPosition(0.5);
                sleep(500);
                flip_servo.setPosition(0);

            }


        }

    }

}
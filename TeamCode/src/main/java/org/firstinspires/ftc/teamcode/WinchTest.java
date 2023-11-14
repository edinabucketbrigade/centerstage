package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class WinchTest extends LinearOpMode {
    public static double WINCH_MOTOR_SPEED = 1;
    public static double WINCH_SERVO_UP_POSITION = 0.5;
    public static double WINCH_SERVO_DOWN_POSITION = 0.05;
    private DcMotor winchMotor;
    private Servo winchServo;

    @Override
    public void runOpMode() {
        winchMotor = hardwareMap.get(DcMotor.class, "winch_motor");

        winchServo = hardwareMap.get(Servo.class, "winch_servo");

        winchMotor.setDirection(DcMotor.Direction.FORWARD);

        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                winchServo.setPosition(WINCH_SERVO_UP_POSITION);
            }
            else if (gamepad1.a) {
                winchServo.setPosition(WINCH_SERVO_DOWN_POSITION);
            }

            if (gamepad1.dpad_up) {
                winchMotor.setPower(-WINCH_MOTOR_SPEED);
            }
            else if (gamepad1.dpad_down) {
                winchMotor.setPower(WINCH_MOTOR_SPEED);
            }
            else {
                winchMotor.setPower(0);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Winch Motor Power", "%.2f", winchMotor.getPower());
            telemetry.addData("Winch Servo Target", "%.2f", winchServo.getPosition());
            telemetry.update();

        }

    }

}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class WinchTest extends LinearOpMode {
    public static double WINCH_MOTOR_SPEED = 1;
    private DcMotor winchMotor;
    private Servo winchServo;


    @Override
    public void runOpMode() {
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        winchMotor = hardwareMap.get(DcMotor.class, "winch_motor");

        winchServo = hardwareMap.get(Servo.class, "winch_servo");

        winchMotor.setDirection(DcMotor.Direction.FORWARD);

        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.y) {
                winchServo.setPosition(1);
            }
            else if (currentGamepad.a) {
                winchServo.setPosition(-1);
            }
            else {
                winchServo.setPosition(0);
            }

            if (currentGamepad.dpad_up) {
                winchMotor.setPower(-WINCH_MOTOR_SPEED);
            }
            else if (currentGamepad.dpad_down) {
                winchMotor.setPower(WINCH_MOTOR_SPEED);
            }
            else {
                winchMotor.setPower(0);
            }


            telemetry.addData("Status", "Running");
            telemetry.addData("Winch Power", "%.2f", winchMotor.getPower());
            telemetry.addData("Winch Servo Target", "%.2f", winchServo.getPosition());
            telemetry.update();

        }

    }

}
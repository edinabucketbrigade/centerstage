package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
@TeleOp
public class TeleOpK extends LinearOpMode {
    public static int targetArmPosition = 0;

    public static double power = 0.1;

    private DcMotorEx armMotor;

    private TouchSensor touchSensor;
    private boolean isReady;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean isPressed = touchSensor.isPressed();

            if (isPressed && !isReady) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                isReady = true;
            }

            int currentArmPosition = armMotor.getCurrentPosition();

            if(isReady) {
                armMotor.setTargetPosition(targetArmPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(power);
            }
            telemetry.addData("Status", "Running");
            telemetry.addData("Ready", isReady);
            telemetry.addData("Current Arm Position", currentArmPosition);
            telemetry.addData("Target Arm Position", targetArmPosition);
            telemetry.addData("Power", power);

            telemetry.update();

        }

        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
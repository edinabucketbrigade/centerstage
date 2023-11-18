package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
@TeleOp
@Disabled
public class TeleOpL extends LinearOpMode {
    public static int targetArmPosition = 0;

    public static double power = 0.1;

    private DcMotorEx armMotor;

    private TouchSensor touchSensor;
    private boolean isReady;
    public static int ARM_MINIMUM = 0;
    public static int ARM_MAXIMUM = 1200;
    public static double ARM_RAISE_POWER = 0.8;
    public static double ARM_LOWER_POWER = 0.4;

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

        int armPosition = ARM_MINIMUM;

        waitForStart();

        boolean previousY = false;

        while (opModeIsActive()) {
            boolean currentY = gamepad1.y;

            boolean isPressed = touchSensor.isPressed();

            if (isPressed && !isReady) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                isReady = true;
            }

            if(isReady) {
                if (currentY && !previousY) {
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

            int currentArmPosition = armMotor.getCurrentPosition();

            telemetry.addData("Status", "Running");
            telemetry.addData("Ready", isReady);
            telemetry.addData("Current Arm Position", currentArmPosition);
            telemetry.addData("Target Arm Position", targetArmPosition);
            telemetry.addData("Power", power);

            telemetry.update();

            previousY = currentY;
        }

        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
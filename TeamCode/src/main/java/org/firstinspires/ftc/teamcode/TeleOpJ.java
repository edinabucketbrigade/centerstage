package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class TeleOpJ extends LinearOpMode {

    // PIDF Loops & Arm Control | FTC | 16379 KookyBotz
    // https://www.youtube.com/watch?v=E6H6Nqe6qJo

    // goBilda 5202-002-0051
    // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-50-9-1-ratio-117-rpm-3-3-5v-encoder/
    private static final double TICKS_PER_REVOLUTION = 1425.1;

    private static final double DEGREES_PER_REVOLUTION = 360;

    private static final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / DEGREES_PER_REVOLUTION;

    private static final double INITIAL_DEGREES_BELOW_HORIZONTAL = 11;

    PIDController controller;

    public static double p = 0.02, i = 0, d = 0.001;

    public static double f = 0.11;

    public static int targetArmPosition = 0;

    private DcMotorEx armMotor;

    @Override
    public void runOpMode() {

        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");

        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            controller.setPID(p, i, d);
            int actualArmPosition = armMotor.getCurrentPosition();
            double actualArmDegrees = getDegrees(actualArmPosition);
            double pid = controller.calculate(actualArmPosition, targetArmPosition);
            double targetArmDegrees = getDegrees(targetArmPosition);
            double targetArmRadians = Math.toRadians(targetArmDegrees);
            double feedForward = Math.cos(targetArmRadians) * f;

            double power = pid + feedForward;

            armMotor.setPower(power);

            telemetry.addData("Status", "Running");
            telemetry.addData("PID", pid);
            telemetry.addData("Feed Forward", feedForward);
            telemetry.addData("Power", power);
            telemetry.addData("Actual Arm Position", actualArmPosition);
            telemetry.addData("Actual Arm Degrees", actualArmDegrees);
            telemetry.addData("Target Arm Position", targetArmPosition);
            telemetry.addData("Target Arm Degrees", targetArmDegrees);

            telemetry.update();

        }

    }

    private double getDegrees(double ticks) {
        double degrees = ticks / TICKS_PER_DEGREE - INITIAL_DEGREES_BELOW_HORIZONTAL;
        return degrees;
    }

}
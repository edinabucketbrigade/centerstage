package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class TeleOpJ extends LinearOpMode {

    // PIDF Loops & Arm Control | FTC | 16379 KookyBotz
    // https://www.youtube.com/watch?v=E6H6Nqe6qJo

    // 0 degrees = arm is parallel to ground
    // 90 degrees = arm is pointing straight up

    public static double TICKS_PER_DEGREE = 4.25; // Determined experimentally
    public static double INITIAL_DEGREES_BELOW_HORIZONTAL = 27; // Determined experimentally
    public static double P = 0.001, I = 0, D = 0.0001;
    public static double F = -0.11;
    public static int TARGET_ARM_POSITION = 0;
    public static boolean MOVE_ARM = false;

    private DcMotorEx armMotor;
    private PIDController controller;

    @Override
    public void runOpMode() {

        controller = new PIDController(P, I, D);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");

        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            controller.setPID(P, I, D);
            int actualArmPosition = armMotor.getCurrentPosition();
            double actualArmDegrees = getDegrees(actualArmPosition);
            double pid = controller.calculate(actualArmPosition, TARGET_ARM_POSITION);
            double targetArmDegrees = getDegrees(TARGET_ARM_POSITION);
            double targetArmRadians = Math.toRadians(targetArmDegrees);
            double feedForward = Math.cos(targetArmRadians) * F;

            double power = pid + feedForward;

            if(MOVE_ARM) {
                armMotor.setPower(power);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("PID", pid);
            telemetry.addData("Feed Forward", feedForward);
            telemetry.addData("Power", power);
            telemetry.addData("Actual Arm Position", actualArmPosition);
            telemetry.addData("Actual Arm Degrees", actualArmDegrees);
            telemetry.addData("Target Arm Position", TARGET_ARM_POSITION);
            telemetry.addData("Target Arm Degrees", targetArmDegrees);

            telemetry.update();

        }

    }

    private double getDegrees(double ticks) {
        double degrees = ticks / TICKS_PER_DEGREE - INITIAL_DEGREES_BELOW_HORIZONTAL;
        return degrees;
    }

}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@TeleOp
public class TeleOpJ extends LinearOpMode {

    // PIDF Loops & Arm Control | FTC | 16379 KookyBotz
    // https://www.youtube.com/watch?v=E6H6Nqe6qJo

    private static final double TICKS_PER_DEGREE = DriveConstants.TICKS_PER_REV / 360;

    PIDController controller;

    public static double p = 0, i = 0, d = 0;

    public static double f = 0;

    public static int targetArmPosition = 0;

    private DcMotorEx armMotor;

    @Override
    public void runOpMode() {

        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            controller.setPID(p, i, d);
            int actualArmPosition = armMotor.getCurrentPosition();
            double pid = controller.calculate(actualArmPosition, targetArmPosition);
            double feedForward = Math.cos(Math.toRadians(targetArmPosition / TICKS_PER_DEGREE)) * f;

            double power = pid + feedForward;

            armMotor.setPower(power);

            telemetry.addData("Status", "Running");
            telemetry.addData("PID", pid);
            telemetry.addData("Feed Forward", feedForward);
            telemetry.addData("Power", power);
            telemetry.addData("Actual Arm Position", actualArmPosition);
            telemetry.addData("Target Arm Position", targetArmPosition);

            telemetry.update();

        }

    }

}
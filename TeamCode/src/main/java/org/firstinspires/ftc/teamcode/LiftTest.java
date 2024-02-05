package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class LiftTest extends LinearOpMode {

    public static double POWER = 0.2;

    @Override
    public void runOpMode() {

        FtcDashboard.getInstance();

        DcMotor leftMotor = hardwareMap.get(DcMotor.class,"left_lift_motor");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class,"right_lift_motor");

        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.y) {
                leftMotor.setPower(POWER);
                rightMotor.setPower(POWER);
            }
            else if(gamepad1.a) {
                leftMotor.setPower(-POWER);
                rightMotor.setPower(-POWER);
            }
            else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }

}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TeleOpF extends LinearOpMode {
    private DcMotor motor = null;

    @Override
    public void runOpMode() {
        motor  = hardwareMap.get(DcMotor.class, "parallel_encoder");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            int position = motor.getCurrentPosition();
            telemetry.addData("Encoder Position", position);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }

}
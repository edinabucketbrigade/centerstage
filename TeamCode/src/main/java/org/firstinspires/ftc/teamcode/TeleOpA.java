package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpA extends LinearOpMode {

    Servo servo;
    double position = (MAX_POS - MIN_POS) / 2;
    boolean rampUp = true;

    @Override
    public void runOpMode() {
        //turn servo 90
        servo = new hardwareMap.get(Servo);
        servo.turn(90);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }

}
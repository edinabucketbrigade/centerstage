package org.firstinspires.ftc.teamcode;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpA extends LinearOpMode {

    Servo servo;
    double position = servo.getPosition();
    double servoOpenPosition = 90;

    @Override
    public void runOpMode() {
        //turn servo 90
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(servoOpenPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }

}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class TeleOpS extends LinearOpMode {
    /*
    Control Hub Portal
        Control Hub
            Motors
                0 - GoBILDA 5201 series - left_lift_motor (has encoder) **Adapter was taken for arm**
                1 - GoBILDA 5201 series - front_encoder (is encoder)
                2 - GoBILDA 5201 series - left_back_drive (encoder port returns 0 and -1)
                3 - GoBILDA 5201 series - left_front_drive (left encoder)
            Digital Devices
                6 - Digital Device - green_left_led
                7 - Digital Device - red_left_led
            Servos
                0 - Servo - left_claw_servo
                1 - Servo - right_claw_servo
                2 - Servo - claw_flip_servo
                3 - Servo - intake_servo
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - arm_motor (has encoder) **Adapter was taken from left lift motor**
                1 - GoBILDA 5201 series - right_lift_motor (has encoder)
                2 - GoBILDA 5201 series - right_front_drive (encoder port has bent pin)
                3 - GoBILDA 5201 series - right_back_drive (right encoder)
            Digital Devices
                3 - REV Touch Sensor - lift_touch
            I2C
                1 - REV 2m Distance Sensor - arm_up_distance
                2 - REV 2m Distance Sensor - arm_down_distance
            Servos
                0 - Servo - right_grip_servo
                1 - Servo - elbow_servo
                2 - Servo - left_grip_servo
                3 - Servo - wrist_servo
    Webcam 1
    */

    public static double ARM_POWER = 0.1;

    private DcMotor armMotor;
    private DistanceSensor armUpDistance;
    private DistanceSensor armDownDistance;
    private DigitalChannel greenLeftLed;
    private DigitalChannel redLeftLed;

    @Override
    public void runOpMode() {
        FtcDashboard.getInstance();

        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armUpDistance = hardwareMap.get(Rev2mDistanceSensor.class, "arm_up_distance");
        armDownDistance = hardwareMap.get(Rev2mDistanceSensor.class, "arm_down_distance");

        greenLeftLed = hardwareMap.get(DigitalChannel.class, "green_left_led");
        redLeftLed = hardwareMap.get(DigitalChannel.class, "red_left_led");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Arm Up Distance Sensor", String.format("%.01f mm", armUpDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("Arm Down Distance Sensor", String.format("%.01f mm", armDownDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("Arm Motor Position", armMotor.getCurrentPosition());
            telemetry.update();

            greenLeftLed.setMode(DigitalChannel.Mode.OUTPUT);
            redLeftLed.setMode(DigitalChannel.Mode.OUTPUT);

            if (gamepad1.y) {
                armMotor.setPower(0.1);
            }
            else if (gamepad1.a) {
                armMotor.setPower(-0.1);
            }
            else {
                armMotor.setPower(0);
            }

            if (gamepad1.x) {
                greenLeftLed.setState(true);
                redLeftLed.setState(false);
            }

            if (gamepad1.b) {
                greenLeftLed.setState(false);
                redLeftLed.setState(true);
            }

        }

    }

}
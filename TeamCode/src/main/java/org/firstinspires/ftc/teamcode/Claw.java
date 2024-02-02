package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Claw {

    public static double RIGHT_OPEN = 0.32;
    public static double RIGHT_CLOSED = 0.5;
    public static double LEFT_OPEN = 0.71;
    public static double LEFT_CLOSED = 0.5;
    public static double PIXEL_MILLIMETERS_THRESHOLD = 10;
    public static double WRIST_UP_POSITION = 0.92;
    public static double WRIST_DOWN_POSITION = 0.27;

    private LinearOpMode opMode;
    private Servo leftServo;
    private Servo rightServo;
    private Servo wristServo;
    private NormalizedColorSensor leftColor;
    private NormalizedColorSensor rightColor;
    public boolean isRightOpen = true;
    public boolean isLeftOpen = true;
    private boolean isWristDown;
    private DigitalChannel greenLeftLed;
    private DigitalChannel redLeftLed;
    private DigitalChannel greenRightLed;
    private DigitalChannel redRightLed;

    // Initializes this.
    public Claw(LinearOpMode opMode) {

        // Remember the op mode.
        this.opMode = opMode;

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get the hardware.
        leftServo = hardwareMap.get(Servo.class,"left_claw_servo");
        rightServo = hardwareMap.get(Servo.class,"right_claw_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        leftColor = hardwareMap.get(NormalizedColorSensor.class, "left_claw_color");
        rightColor = hardwareMap.get(NormalizedColorSensor.class, "right_claw_color");
        greenLeftLed = hardwareMap.get(DigitalChannel.class, "green_left_led");
        redLeftLed = hardwareMap.get(DigitalChannel.class, "red_left_led");
        greenRightLed = hardwareMap.get(DigitalChannel.class, "green_right_led");
        redRightLed = hardwareMap.get(DigitalChannel.class, "red_right_led");

        // Initialize hardware.
        greenLeftLed.setMode(DigitalChannel.Mode.OUTPUT);
        redLeftLed.setMode(DigitalChannel.Mode.OUTPUT);
        greenRightLed.setMode(DigitalChannel.Mode.OUTPUT);
        redRightLed.setMode(DigitalChannel.Mode.OUTPUT);

    }

    // Update this.
    public void update() {

        // Get the claw distance sensors.
        DistanceSensor leftDistance = (DistanceSensor) leftColor;
        DistanceSensor rightDistance = (DistanceSensor) rightColor;

        // Get the claw distances.
        double leftMillimeters = leftDistance.getDistance(DistanceUnit.MM);
        double rightMillimeters = rightDistance.getDistance(DistanceUnit.MM);

        // If there is a pixel in the left claw...
        if(leftMillimeters < PIXEL_MILLIMETERS_THRESHOLD) {

            // Close the left claw.
            closeLeft();

        }

        // If there is a pixel in the right claw...
        if(rightMillimeters < PIXEL_MILLIMETERS_THRESHOLD) {

            // Close the right claw.
            closeRight();

        }

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Add claw information to the telemetry.
        telemetry.addData("Claw", "Left Open = %b, Right Open = %b, Wrist Down = %b, Left Distance = %.0f mm, Right Distance = %.0f mm", isLeftOpen, isRightOpen, isWristDown, leftMillimeters, rightMillimeters);

    }

    // Toggles the claws.
    public void toggle() {

        // Toggle the left claw.
        toggleLeft();

        // Toggle the right claw.
        toggleRight();

    }

    // Toggles the left claw.
    public void toggleLeft() {

        // Toggle the left claw.
        if (isLeftOpen) {
            closeLeft();
        } else {
            openLeft();
        }

    }

    // Toggles the right claw.
    public void toggleRight() {

        // Toggle the right claw.
        if (isRightOpen) {
            closeRight();
        } else {
            openRight();
        }

    }

    // Closes the left claw.
    public void closeLeft() {

        // Make the left LED green.
        setLedGreen(greenLeftLed, redLeftLed);

        // Close the left claw.
        leftServo.setPosition(LEFT_CLOSED);

        // Remember that the left claw is closed.
        isLeftOpen = false;

    }

    // Opens the left claw.
    public void openLeft() {

        // Make the left LED red.
        setLedRed(greenLeftLed, redLeftLed);

        // Open the left claw.
        leftServo.setPosition(LEFT_OPEN);

        // Remember that the left claw is open.
        isLeftOpen = true;

    }

    // Closes the right claw.
    public void closeRight() {

        // Make the right LED green.
        setLedGreen(greenRightLed, redRightLed);

        // Close the right claw.
        rightServo.setPosition(RIGHT_CLOSED);

        // Remember that the right claw is closed.
        isRightOpen = false;

    }

    // Opens the right claw.
    public void openRight() {

        // Make the right LED red.
        setLedRed(greenRightLed, redRightLed);

        // Open the right claw.
        rightServo.setPosition(RIGHT_OPEN);

        // Remember that the right claw is open.
        isRightOpen = true;

    }

    // Closes the claw.
    public void close() {

        // Close the left claw.
        closeLeft();

        // Close the right claw.
        closeRight();

    }

    // Opens the claw.
    public void open() {

        // Open the left claw.
        openLeft();

        // Open the right claw.
        openRight();

    }

    // Toggles the wrist.
    public void toggleWrist() {

        // Toggle the wrist.
        if (isWristDown) {
            raiseWrist();
        } else {
            lowerWrist();
        }

    }

    // Raises the wrist.
    public void raiseWrist() {

        // Raise the wrist.
        wristServo.setPosition(WRIST_UP_POSITION);

        // Remember that the wrist is up.
        isWristDown = false;

    }

    // Lowers the wrist.
    public void lowerWrist() {

        // Lowers the wrist.
        wristServo.setPosition(WRIST_DOWN_POSITION);

        // Remember that the wrist is down.
        isWristDown = true;

    }

    // Sets an LED color to green.
    private void setLedGreen(DigitalChannel greenLed, DigitalChannel redLed) {

        // Set the LED color to green.
        greenLed.setState(true);
        redLed.setState(false);

    }

    // Sets an LED color to red.
    private void setLedRed(DigitalChannel greenLed, DigitalChannel redLed) {

        // Set the LED color to red.
        greenLed.setState(false);
        redLed.setState(true);

    }

}

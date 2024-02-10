package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.WristPosition.BACKDROP;
import static org.firstinspires.ftc.teamcode.WristPosition.GROUND;
import static org.firstinspires.ftc.teamcode.WristPosition.RELEASE;

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

    public static double LEFT_CLOSED = 0.35;
    public static double LEFT_PARTIALLY_OPEN = 0.455;
    public static double LEFT_FULLY_OPEN = 0.67;
    public static double PIXEL_CAPTURE_THRESHOLD = 35;
    public static double RIGHT_CLOSED = 0.6;
    public static double RIGHT_PARTIALLY_OPEN = 0.52;
    public static double RIGHT_FULLY_OPEN = 0.3;
    public static double WRIST_GROUND_POSITION = 0.118;
    public static double WRIST_BACKDROP_POSITION = 0.76;
    public static double WRIST_RELEASE_POSITION = 0.88;

    private RobotHardwareC robotHardware;
    private Servo leftServo;
    private Servo rightServo;
    private Servo wristServo;
    private NormalizedColorSensor leftColor;
    private NormalizedColorSensor rightColor;
    public boolean isRightOpen = true;
    public boolean isLeftOpen = true;
    private WristPosition wristPosition;
    private DigitalChannel greenLeftLed;
    private DigitalChannel redLeftLed;
    private DigitalChannel greenRightLed;
    private DigitalChannel redRightLed;
    private boolean automaticallyCaptureLeft = true;
    private boolean automaticallyCaptureRight = true;

    // Initializes this.
    public Claw(RobotHardwareC robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

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

        // If the robot is automatically driving...
        if(robotHardware.isAutomaticallyDriving()) {

            // Exit the method.
            return;

        }

        // Get the claw distance sensors.
        DistanceSensor leftDistance = (DistanceSensor) leftColor;
        DistanceSensor rightDistance = (DistanceSensor) rightColor;

        // Get the claw distances.
        double leftMillimeters = leftDistance.getDistance(DistanceUnit.MM);
        double rightMillimeters = rightDistance.getDistance(DistanceUnit.MM);

        // If there is a pixel in the left claw...
        if(leftMillimeters < PIXEL_CAPTURE_THRESHOLD) {

            // If we are automatically capturing left pixels...
            if(automaticallyCaptureLeft) {

                // Close the left claw.
                closeLeft();

                // Disable automatically capturing left pixels for now.
                automaticallyCaptureLeft = false;

            }

        }

        // Otherwise (if there is no pixel in the left claw)...
        else {

            // Enable automatically capturing left pixels.
            automaticallyCaptureLeft = true;

            // Open the left claw.
            //openLeftFully();

        }

        // If there is a pixel in the right claw...
        if(rightMillimeters < PIXEL_CAPTURE_THRESHOLD) {

            // If we are automatically capturing right pixels...
            if(automaticallyCaptureRight) {

                // Close the right claw.
                closeRight();

                // Disable automatically capturing right pixels for now.
                automaticallyCaptureRight = false;

            }

        }

        // Otherwise (if there is no pixel in the right claw)...
        else {

            // Enable automatically capturing right pixels.
            automaticallyCaptureRight = true;

            // Open the right claw.
            //openRightFully();

        }


        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Add claw information to the telemetry.
        telemetry.addData("Claw", "Left Open = %b, Right Open = %b, Wrist Position = %s, Left Distance = %.0f mm, Right Distance = %.0f mm, Capture Left = %b, Capture Right = %b", isLeftOpen, isRightOpen, wristPosition, leftMillimeters, rightMillimeters, automaticallyCaptureLeft, automaticallyCaptureRight);

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
            openLeftFully();
        }

    }

    // Toggles the right claw.
    public void toggleRight() {

        // Toggle the right claw.
        if (isRightOpen) {
            closeRight();
        } else {
            openRightFully();
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

    public void openLeftPartially() {
        setLedRed(greenLeftLed, redLeftLed);

        leftServo.setPosition(LEFT_PARTIALLY_OPEN);

        isLeftOpen = true;
}

    // Opens the left claw.
    public void openLeftFully() {

        // Make the left LED red.
        setLedRed(greenLeftLed, redLeftLed);

        // Open the left claw.
        leftServo.setPosition(LEFT_FULLY_OPEN);

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

    public void openRightPartially() {
        setLedRed(greenRightLed, redRightLed);

        rightServo.setPosition(RIGHT_PARTIALLY_OPEN);

        isRightOpen = true;
    }

    // Opens the right claw.
    public void openRightFully() {

        // Make the right LED red.
        setLedRed(greenRightLed, redRightLed);

        // Open the right claw.
        rightServo.setPosition(RIGHT_FULLY_OPEN);

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
    public void openFully() {

        // Open the left claw.
        openLeftFully();

        // Open the right claw.
        openRightFully();

    }

    public void openPartially(){
        openLeftPartially();
        openRightPartially();
    }

    // Toggles the wrist.
    public void toggleWrist() {

        // Toggle the wrist.
        if (wristPosition == GROUND) {
            setWristBackdrop();
        } else {
            setWristGround();
        }

    }

    // Sets the wrist position to backdrop.
    public void setWristBackdrop() {

        // Set the wrist position to backdrop.
        wristServo.setPosition(WRIST_BACKDROP_POSITION);

        // Remember the wrist position.
        wristPosition = BACKDROP;

    }

    // Sets the wrist position to release.
    public void setWristRelease() {

        // Set the wrist position to release..
        wristServo.setPosition(WRIST_RELEASE_POSITION);

        // Remember the wrist position.
        wristPosition = RELEASE;

    }

    // Sets the wrist position to ground.
    public void setWristGround() {

        // Set the wrist position to ground.
        wristServo.setPosition(WRIST_GROUND_POSITION);

        // Remember the wrist position.
        wristPosition = GROUND;

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

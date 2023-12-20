package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
@Config
public class CameraPixelColor extends OpenCvPipeline {
    public static int MINIMUM_WHITE_VALUES = 150;
    public static int MINIMUM_YELLOW_VALUES = 100;
    public static int MINIMUM_GREEN_VALUES = 40;
    public static int MINIMUM_PURPLE_VALUES = 40;
    public static int MAXIMUM_WHITE_VALUES = 255;
    public static int MAXIMUM_YELLOW_VALUES = 255;
    public static int MAXIMUM_GREEN_VALUES = 255;
    public static int MAXIMUM_PURPLE_VALUES = 255;
    public static int MINIMUM_WHITE_SATURATION = 0;
    public static int MAXIMUM_WHITE_SATURATION = 20;
    public static int MINIMUM_WHITE_HUE = 0;
    public static int MINIMUM_YELLOW_HUE = 15;
    public static int MINIMUM_GREEN_HUE = 50;
    public static int MINIMUM_PURPLE_HUE = 115;
    public static int MAXIMUM_WHITE_HUE = 255;
    public static int MAXIMUM_YELLOW_HUE = 25;
    public static int MAXIMUM_GREEN_HUE = 90;
    public static int MAXIMUM_PURPLE_HUE = 140;

    public static Rect LEFT_ROI = new Rect(30,160, 150,120);
    public static Rect RIGHT_ROI = new Rect(435,165, 150,120);
    public enum Location{
        Left,
        Right,
    }
    private Location location;

    Telemetry telemetry;
    Mat matWhite = new Mat();
    Mat matGreen = new Mat();
    Mat matYellow = new Mat();
    Mat matPurple = new Mat();

    public Scalar MINIMUM_WHITE;
    public Scalar MAXIMUM_WHITE;
    public Scalar MINIMUM_YELLOW;
    public Scalar MAXIMUM_YELLOW;
    public Scalar MINIMUM_GREEN;
    public Scalar MAXIMUM_GREEN;
    public Scalar MINIMUM_PURPLE;
    public Scalar MAXIMUM_PURPLE;
    RevBlinkinLedDriver blinkinLedDriver1;
    RevBlinkinLedDriver blinkinLedDriver2;
    RevBlinkinLedDriver.BlinkinPattern blinkinPattern1;
    RevBlinkinLedDriver.BlinkinPattern blinkinPattern2;
    public CameraPixelColor(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {

        MINIMUM_WHITE = new Scalar(MINIMUM_WHITE_HUE, MINIMUM_WHITE_SATURATION, MINIMUM_WHITE_VALUES);
        MAXIMUM_WHITE = new Scalar(MAXIMUM_WHITE_HUE, MAXIMUM_WHITE_SATURATION, MAXIMUM_WHITE_VALUES);
        MINIMUM_YELLOW = new Scalar(MINIMUM_YELLOW_HUE, MINIMUM_YELLOW_VALUES, MINIMUM_YELLOW_VALUES);
        MAXIMUM_YELLOW = new Scalar(MAXIMUM_YELLOW_HUE, MAXIMUM_YELLOW_VALUES, MAXIMUM_YELLOW_VALUES);
        MINIMUM_GREEN = new Scalar(MINIMUM_GREEN_HUE, MINIMUM_GREEN_VALUES, MINIMUM_GREEN_VALUES);
        MAXIMUM_GREEN = new Scalar(MAXIMUM_GREEN_HUE, MAXIMUM_GREEN_VALUES, MAXIMUM_GREEN_VALUES);
        MINIMUM_PURPLE = new Scalar(MINIMUM_PURPLE_HUE, MINIMUM_PURPLE_VALUES, MINIMUM_PURPLE_VALUES);
        MAXIMUM_PURPLE = new Scalar(MAXIMUM_PURPLE_HUE, MAXIMUM_PURPLE_VALUES, MAXIMUM_PURPLE_VALUES);

        blinkinLedDriver1 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin1");
        blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver1.setPattern(blinkinPattern1);

        blinkinLedDriver2 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin2");
        blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver2.setPattern(blinkinPattern2);

        Imgproc.cvtColor(input, matWhite, Imgproc.COLOR_RGB2HSV);
        Core.inRange(matWhite, MINIMUM_WHITE, MAXIMUM_WHITE, matWhite);
        Imgproc.cvtColor(input, matYellow, Imgproc.COLOR_RGB2HSV);
        Core.inRange(matYellow, MINIMUM_YELLOW, MAXIMUM_YELLOW, matYellow);
        Imgproc.cvtColor(input, matGreen, Imgproc.COLOR_RGB2HSV);
        Core.inRange(matGreen, MINIMUM_GREEN, MAXIMUM_GREEN, matGreen);
        Imgproc.cvtColor(input, matPurple, Imgproc.COLOR_RGB2HSV);
        Core.inRange(matPurple, MINIMUM_PURPLE, MAXIMUM_PURPLE, matPurple);

        Mat leftWhite = matWhite.submat(LEFT_ROI);
        Mat rightWhite = matWhite.submat(RIGHT_ROI);
        Mat leftYellow = matYellow.submat(LEFT_ROI);
        Mat rightYellow = matYellow.submat(RIGHT_ROI);
        Mat leftGreen = matGreen.submat(LEFT_ROI);
        Mat rightGreen = matGreen.submat(RIGHT_ROI);
        Mat leftPurple = matPurple.submat(LEFT_ROI);
        Mat rightPurple = matPurple.submat(RIGHT_ROI);

        double leftValueWhite = Core.sumElems(leftWhite).val[0];
        double rightValueWhite = Core.sumElems(rightWhite).val[0];
        double leftValueYellow = Core.sumElems(leftYellow).val[0];
        double rightValueYellow = Core.sumElems(rightYellow).val[0];
        double leftValueGreen = Core.sumElems(leftGreen).val[0];
        double rightValueGreen = Core.sumElems(rightGreen).val[0];
        double leftValuePurple = Core.sumElems(leftPurple).val[0];
        double rightValuePurple = Core.sumElems(rightPurple).val[0];

        leftWhite.release();
        rightWhite.release();
        leftYellow.release();
        rightYellow.release();
        leftGreen.release();
        rightGreen.release();
        leftPurple.release();
        rightPurple.release();

        telemetry.addData("Left White Raw Value", leftValueWhite);
        telemetry.addData("Right White Raw Value", rightValueWhite);
        telemetry.addData("Left Green Raw Value", leftValueGreen);
        telemetry.addData("Right Green Raw Value", rightValueGreen);
        telemetry.addData("Left Purple Raw Value", leftValuePurple);
        telemetry.addData("Right Purple Raw Value", leftValuePurple);
        telemetry.addData("Left Yellow Raw Value", leftValueYellow);
        telemetry.addData("Right Yellow Raw Value", rightValueYellow);

        if (leftValueWhite > leftValueYellow && leftValueWhite > leftValueGreen && leftValueWhite > leftValuePurple)
            blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        else if(leftValueYellow > leftValueWhite && leftValueYellow > leftValueGreen && leftValueYellow > leftValuePurple)
            blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        else if(leftValueGreen > leftValueWhite && leftValueGreen > leftValueYellow && leftValueGreen > leftValuePurple)
            blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        else if(leftValuePurple > leftValueWhite && leftValuePurple > leftValueYellow && leftValuePurple > leftValueGreen)
            blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        else
            blinkinPattern1 = RevBlinkinLedDriver.BlinkinPattern.RED;
        if (rightValueWhite > rightValueYellow && rightValueWhite > rightValueGreen && rightValueWhite > rightValuePurple)
            blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        else if(rightValueYellow > rightValueWhite && rightValueYellow > rightValueGreen && rightValueYellow > rightValuePurple)
            blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        else if(rightValueGreen > rightValueWhite && rightValueGreen > rightValueYellow && rightValueGreen > rightValuePurple)
            blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        else if(rightValuePurple > rightValueWhite && rightValuePurple > rightValueYellow && rightValuePurple > rightValueGreen)
            blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        else
            blinkinPattern2 = RevBlinkinLedDriver.BlinkinPattern.RED;

        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar pixelColor = new Scalar(255,255,255);

        Imgproc.rectangle(matWhite, LEFT_ROI, pixelColor);
        Imgproc.rectangle(matWhite, RIGHT_ROI, pixelColor);
        Imgproc.rectangle(matPurple, LEFT_ROI, pixelColor);
        Imgproc.rectangle(matPurple, RIGHT_ROI, pixelColor);
        Imgproc.rectangle(matGreen, LEFT_ROI, pixelColor);
        Imgproc.rectangle(matGreen, RIGHT_ROI, pixelColor);
        Imgproc.rectangle(matYellow, LEFT_ROI, pixelColor);
        Imgproc.rectangle(matYellow, RIGHT_ROI, pixelColor);

        blinkinLedDriver2.setPattern(blinkinPattern2);
        telemetry.addData("Right Color Is", blinkinPattern2);
        blinkinLedDriver1.setPattern(blinkinPattern1);
        telemetry.addData("Left Color Is", blinkinPattern1);

        telemetry.update();
        return matWhite;
    }
    public CameraPixelColor.Location getLocation(){return location;}

}
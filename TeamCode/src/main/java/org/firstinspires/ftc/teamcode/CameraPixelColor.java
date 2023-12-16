package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

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
    Mat mat = new Mat();

    public Scalar MINIMUM_WHITE;
    public Scalar MAXIMUM_WHITE;
    public Scalar MINIMUM_YELLOW;
    public Scalar MAXIMUM_YELLOW;
    public Scalar MINIMUM_GREEN;
    public Scalar MAXIMUM_GREEN;
    public Scalar MINIMUM_PURPLE;
    public Scalar MAXIMUM_PURPLE;
    public CameraPixelColor(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        MINIMUM_WHITE = new Scalar(MINIMUM_WHITE_HUE, MINIMUM_WHITE_SATURATION, MINIMUM_WHITE_VALUES);
        MAXIMUM_WHITE = new Scalar(MAXIMUM_WHITE_HUE, MAXIMUM_WHITE_SATURATION, MAXIMUM_WHITE_VALUES);
        MINIMUM_YELLOW = new Scalar(MINIMUM_YELLOW_HUE, MINIMUM_YELLOW_VALUES, MINIMUM_YELLOW_VALUES);
        MAXIMUM_YELLOW = new Scalar(MAXIMUM_YELLOW_HUE, MAXIMUM_YELLOW_VALUES, MAXIMUM_YELLOW_VALUES);
        MINIMUM_GREEN = new Scalar(MINIMUM_GREEN_HUE, MINIMUM_GREEN_VALUES, MINIMUM_GREEN_VALUES);
        MAXIMUM_GREEN = new Scalar(MAXIMUM_GREEN_HUE, MAXIMUM_GREEN_VALUES, MAXIMUM_GREEN_VALUES);
        MINIMUM_PURPLE = new Scalar(MINIMUM_PURPLE_HUE, MINIMUM_PURPLE_VALUES, MINIMUM_PURPLE_VALUES);
        MAXIMUM_PURPLE = new Scalar(MAXIMUM_PURPLE_HUE, MAXIMUM_PURPLE_VALUES, MAXIMUM_PURPLE_VALUES);

//        Core.inRange(mat, MINIMUM_WHITE, MAXIMUM_WHITE, mat);
//        Core.inRange(mat, MINIMUM_YELLOW, MAXIMUM_YELLOW, mat);
//        Core.inRange(mat, MINIMUM_GREEN, MAXIMUM_GREEN, mat);
//        Core.inRange(mat, MINIMUM_PURPLE, MAXIMUM_PURPLE, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0];
        double rightValue = Core.sumElems(right).val[0];

        left.release();
        right.release();

        if (rightValue >= leftValue) 
            location = Location.Right;
        else
            location = Location.Left;
        telemetry.addData("Left Raw Value", leftValue);
        telemetry.addData("Right Raw Value", rightValue);

        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar pixelColor = new Scalar(255,255,255);

        Imgproc.rectangle(mat, LEFT_ROI, pixelColor);
        Imgproc.rectangle(mat, RIGHT_ROI, pixelColor);

        return mat;
    }
    public CameraPixelColor.Location getLocation(){return location;}

}
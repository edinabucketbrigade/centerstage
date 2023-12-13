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
    public static int MINIMUM_WHITE_VALUES = 0;
    public static int MINIMUM_YELLOW_VALUES = 0;
    public static int MINIMUM_GREEN_VALUES = 0;
    public static int MINIMUM_PURPLE_VALUES = 0;
    public static int MAXIMUM_WHITE_VALUES = 160;
    public static int MAXIMUM_YELLOW_VALUES = 0;
    public static int MAXIMUM_GREEN_VALUES = 0;
    public static int MAXIMUM_PURPLE_VALUES = 0;
    public static int MINIMUM_WHITE_HUE = 0;
    public static int MINIMUM_YELLOW_HUE = 0;
    public static int MINIMUM_GREEN_HUE = 0;
    public static int MINIMUM_PURPLE_HUE = 0;
    public static int MAXIMUM_WHITE_HUE = 20;
    public static int MAXIMUM_YELLOW_HUE = 0;
    public static int MAXIMUM_GREEN_HUE = 0;
    public static int MAXIMUM_PURPLE_HUE = 0;

    public static Rect LEFT_ROI = new Rect(30,160, 150,120);
    public static Rect RIGHT_ROI = new Rect(435,165, 150,120);
    public enum Location{
        Left,
        Right,
    }
    private Location location;

    Telemetry telemetry;
    Mat mat = new Mat();
    public CameraPixelColor(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar MINIMUM_WHITE = new Scalar(MINIMUM_WHITE_HUE,MINIMUM_WHITE_VALUES,MINIMUM_WHITE_VALUES);
        Scalar MAXIMUM_WHITE = new Scalar(MAXIMUM_WHITE_HUE,MAXIMUM_WHITE_VALUES,MAXIMUM_WHITE_VALUES);
        Scalar MINIMUM_YELLOW = new Scalar(MINIMUM_YELLOW_HUE,MINIMUM_YELLOW_VALUES,MINIMUM_YELLOW_VALUES);
        Scalar MAXIMUM_YELLOW = new Scalar(MAXIMUM_YELLOW_HUE,MAXIMUM_YELLOW_VALUES,MAXIMUM_YELLOW_VALUES);
        Scalar MINIMUM_GREEN = new Scalar(MINIMUM_GREEN_HUE,MINIMUM_GREEN_VALUES,MINIMUM_GREEN_VALUES);
        Scalar MAXIMUM_GREEN = new Scalar(MAXIMUM_GREEN_HUE,MAXIMUM_GREEN_VALUES,MAXIMUM_GREEN_VALUES);
        Scalar MINIMUM_PURPLE = new Scalar(MINIMUM_PURPLE_HUE,MINIMUM_PURPLE_VALUES,MINIMUM_PURPLE_VALUES);
        Scalar MAXIMUM_PURPLE = new Scalar(MAXIMUM_PURPLE_HUE,MAXIMUM_PURPLE_VALUES,MAXIMUM_PURPLE_VALUES);

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Core.inRange(mat, MINIMUM_WHITE, MAXIMUM_WHITE, mat);
        Core.inRange(mat, MINIMUM_YELLOW, MAXIMUM_YELLOW, mat);
        Core.inRange(mat, MINIMUM_GREEN, MAXIMUM_GREEN, mat);
        Core.inRange(mat, MINIMUM_PURPLE, MAXIMUM_PURPLE, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0];
        double rightValue = Core.sumElems(right).val[0];

        telemetry.addData("Left Raw Value", leftValue);
        telemetry.addData("Right Raw Value", rightValue);

        left.release();
        right.release();

        if (rightValue >= leftValue){
            location = Location.Right;
        } else {
            location = Location.Left;
        }

        return mat;
    }
    public CameraPixelColor.Location getLocation(){return location;}

}
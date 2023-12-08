package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous
public class CameraPixelColor extends OpenCvPipeline {
    public static int MINIMUM_WHITE_VALUES = 0;
    public static int MINIMUM_YELLOW_VALUES = 0;
    public static int MINIMUM_GREEN_VALUES = 0;
    public static int MINIMUM_PURPLE_VALUES = 0;
    public static int MAXIMUM_WHITE_VALUES = 0;
    public static int MAXIMUM_YELLOW_VALUES = 0;
    public static int MAXIMUM_GREEN_VALUES = 0;
    public static int MAXIMUM_PURPLE_VALUES = 0;
    public static int MINIMUM_WHITE_HUE = 0;
    public static int MINIMUM_YELLOW_HUE = 0;
    public static int MINIMUM_GREEN_HUE = 0;
    public static int MINIMUM_PURPLE_HUE = 0;
    public static int MAXIMUM_WHITE_HUE = 0;
    public static int MAXIMUM_YELLOW_HUE = 0;
    public static int MAXIMUM_GREEN_HUE = 0;
    public static int MAXIMUM_PURPLE_HUE = 0;

    Telemetry telemetry;
    Mat mat = new Mat();
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

        return mat;
    }
}

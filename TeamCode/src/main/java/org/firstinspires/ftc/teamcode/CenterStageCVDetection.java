package org.firstinspires.ftc.teamcode;

import static bucketbrigade.casperlibrary.TeamPropLocation.LEFT;
import static bucketbrigade.casperlibrary.TeamPropLocation.MIDDLE;
import static bucketbrigade.casperlibrary.TeamPropLocation.RIGHT;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import bucketbrigade.casperlibrary.TeamPropLocation;

@Config
public class CenterStageCVDetection extends OpenCvPipeline {
    public static double MINIMUM_BLUE_VALUES = 40;
    public static double MINIMUM_RED_VALUES = 100;
    public static double MAXIMUM_VALUES = 255;
    public static double MINIMUM_BLUE_HUE = 100;
    public static double MAXIMUM_BLUE_HUE = 115;
    public static double MINIMUM_RED_LOW_HUE = 0;
    public static double MAXIMUM_RED_LOW_HUE = 25;
    public static double MINIMUM_RED_HIGH_HUE = 160;
    public static double MAXIMUM_RED_HIGH_HUE = 255;
    public static Rect LEFT_ROI = new Rect();
    public static Rect MIDDLE_ROI = new Rect();
    public static Rect RIGHT_ROI = new Rect();
    Telemetry telemetry;
    boolean redAlliance;
    boolean startLeft;
    boolean parkLeft;
    boolean isNew;
    Mat mat = new Mat();

    private TeamPropLocation location;
    /*
     *   ROI is an abbreviation of Region of Interest.
     *   This creates a rectangle of areas in the camera where a game element may be placed
     */

    public CenterStageCVDetection(boolean parkLeft, boolean redAlliance, boolean startLeft, Telemetry telemetry, boolean isNew) {
        this.parkLeft = parkLeft;
        this.redAlliance = redAlliance;
        this.startLeft = startLeft;
        this.telemetry = telemetry;
        this.isNew = isNew;
    }
    @Override
    public Mat processFrame(Mat input) {
        if (!isNew) {

            LEFT_ROI = new Rect(30,160,150,120);
            MIDDLE_ROI = new Rect(225,150,150,120);
            RIGHT_ROI = new Rect(435,165,150,120);

        } else {

            LEFT_ROI = new Rect(30,135,150,120);
            MIDDLE_ROI = new Rect(252,120,150,120);
            RIGHT_ROI = new Rect(475,135,150,120);
        }

        telemetry.addData("Alliance", redAlliance ? "Red" : "Blue");
        telemetry.addData("Start", startLeft ? "Left" : "Right");
        telemetry.addData("Park", parkLeft ? "Left" : "Right");

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar MINIMUM_BLUE = new Scalar(MINIMUM_BLUE_HUE, MINIMUM_BLUE_VALUES, MINIMUM_BLUE_VALUES);
        Scalar MAXIMUM_BLUE = new Scalar(MAXIMUM_BLUE_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);
        Scalar MINIMUM_RED_LOW = new Scalar(MINIMUM_RED_LOW_HUE, MINIMUM_RED_VALUES, MINIMUM_RED_VALUES);
        Scalar MAXIMUM_RED_LOW = new Scalar(MAXIMUM_RED_LOW_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);
        Scalar MINIMUM_RED_HIGH = new Scalar(MINIMUM_RED_HIGH_HUE, MINIMUM_RED_VALUES, MINIMUM_RED_VALUES);
        Scalar MAXIMUM_RED_HIGH = new Scalar(MAXIMUM_RED_HIGH_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);

        if (redAlliance){
            Mat mat1 = mat.clone();
            Mat mat2 = mat.clone();
            Core.inRange(mat1, MINIMUM_RED_LOW, MAXIMUM_RED_LOW, mat1);
            Core.inRange(mat2, MINIMUM_RED_HIGH, MAXIMUM_RED_HIGH, mat2);
            Core.bitwise_or(mat1, mat2, mat);
            mat1.release();
            mat2.release();
        }
        else {
            Core.inRange(mat, MINIMUM_BLUE, MAXIMUM_BLUE, mat);
        }
        //submat = submatrix - portion of original matrix
        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        /*
        *   We can determine the percentage of white in the image by
        *   adding all of the props together and dividing by its area
        *   which averages the submatrices and then dividing everything
        *   by 255 (The highest value for a grayscale prop).
        *
        *   Here we only take the first value of the sum result because
        *   there is only one channel in a grayscale image.
        */
        double leftValue = Core.sumElems(left).val[0];
        double rightValue = Core.sumElems(right).val[0];
        double middleValue = Core.sumElems(middle).val[0];

        telemetry.addData("Left Raw Value", leftValue);
        telemetry.addData("Right Raw Value", rightValue);
        telemetry.addData("Middle Raw Value", middleValue);

        left.release();
        right.release();
        middle.release();

        if(leftValue >= rightValue && leftValue >= middleValue){
            location = LEFT;
        } else if (rightValue >= middleValue) {
            location = RIGHT;
        } else {
            location = MIDDLE;
        }

        telemetry.addData("Location", location);

        telemetry.update();

        /*
        *   We can draw rectangles to visualize the location of the props.
        *   We first convert the grayscale image back to RGB so we can
        *   draw colored rectangles. Then we define two colors to differentiate
        *   the prop from pixels. Finally we draw the rectangles onto the matrices.
        */
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar pixelColor = new Scalar(255,255,255);
        Scalar propColor = new Scalar(0,0,255);
        /*
        *   Here we used a Ternary Operator (Conditional Expression) which is
        *   just a mini if statement that chooses which colors to use based
        *   on the location of the prop.
        */
        Imgproc.rectangle(mat, LEFT_ROI, location == LEFT ? pixelColor:propColor);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == MIDDLE ? pixelColor:propColor);
        Imgproc.rectangle(mat, RIGHT_ROI, location == RIGHT ? pixelColor:propColor);

        return mat;
    }
    /*
    *   Now that we have a detector pipeline we can use it to create an autonomous OpMode
    */
    public TeamPropLocation getLocation(){
        return location;
    }
}

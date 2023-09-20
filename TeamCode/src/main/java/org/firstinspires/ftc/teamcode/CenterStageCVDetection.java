package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CenterStageCVDetection extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location{
        Left,
        Right,
        Not_Found
    }
    private Location location;
    /*
    *   ROI is an abbreviation of Region of Interest.
    *   This creates a rectangle of areas in the camera where a game element may be placed
    */
    static final Rect Left_ROI = new Rect(new Point(60,35),new Point(120,75));
    static final Rect Right_ROI = new Rect(new Point(140,35),new Point(200,75));
    public CenterStageCVDetection(Telemetry t) {
        telemetry = t;
    }
    public double colorPercentThreshold = 0.4;
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV_FULL);
        Scalar lowHSV = new Scalar(215,50,30);
        Scalar highHSV = new Scalar(225,100,100);

        Core.inRange(mat, lowHSV, highHSV, mat);
        //submat = submatrix - portion of original matrix
        Mat left = mat.submat(Left_ROI);
        Mat right = mat.submat(Right_ROI);

        /*
        *   We can determine the percentage of white in the image by
        *   adding all of the props together and dividing by its area
        *   which averages the submatrices and then dividing everything
        *   by 255 (The highest value for a grayscale prop).
        *
        *   Here we only take the first value of the sum result because
        *   there is only one channel in a grayscale image.
        */
        double leftValue = Core.sumElems(left).val[0] / Left_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / Right_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left Raw Value:",(int) Core.sumElems(left).val[0]);
        telemetry.addData("Right Raw Value:",(int) Core.sumElems(right).val[0]);
        telemetry.addData("Left Percentage:",Math.round(leftValue * 100)+"%");
        telemetry.addData("Right Percentage:",Math.round(rightValue * 100)+"%");

        boolean propLeft = leftValue > colorPercentThreshold;
        boolean propRight = rightValue > colorPercentThreshold;

        if(propLeft && propRight){
            //not found
            location = Location.Not_Found;
            telemetry.addData("Prop Location:", "Not Found");
        }
        if(propLeft){
            //prop is on right
            location = Location.Right;
            telemetry.addData("Prop Location:", "Right");
        } else {
            //prop is on left
            location = Location.Left;
            telemetry.addData("Prop Location:", "Left");
        }

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
        Imgproc.rectangle(mat, Left_ROI, location == Location.Left? pixelColor:propColor);
        Imgproc.rectangle(mat, Right_ROI, location == Location.Right? pixelColor:propColor);

        return mat;
    }
    /*
    *   Now that we have a detector pipeline we can use it to create an autonomous OpMode
    */
    public Location getLocation(){
        return location;
    }
}

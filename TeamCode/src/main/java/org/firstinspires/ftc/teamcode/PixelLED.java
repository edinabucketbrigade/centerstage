package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.graphics.ColorSpace;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
@Config
public class PixelLED {
    public enum PixelColor{
        White,
        Yellow,
        Green,
        Purple
    }
    public Mat processFrame(Mat input) {
        ColorSpace.Rgb WHITE_VALUES;
        ColorSpace.Rgb YELLOW_VALUES;
        ColorSpace.Rgb GREEN_VALUES;
        ColorSpace.Rgb PURPLE_VALUES;

    /*
    if (color == whiteColor){

    } else if (color == yellowColor){

    } else if (color == greenColor){

    } else if (color == purpleColor){

    }
     */
    }
}

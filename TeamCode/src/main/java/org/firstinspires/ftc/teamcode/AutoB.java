package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(preselectTeleOp = "TeleOpB")
public class AutoB extends LinearOpMode {

    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        webcam.setPipeline(new CenterStagePipeline());

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }

    class CenterStagePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(input, new Point(input.cols()/4,input.rows()/4), new Point(input.cols()*(3f/4f),input.rows()*(3f/4f)), new Scalar(0, 255, 0), 4);
            return input;
        }
    }

}
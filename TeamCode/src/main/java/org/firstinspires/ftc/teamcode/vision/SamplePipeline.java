package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class SamplePipeline extends OpenCvPipeline {
    public Mat processFrame(Mat input){
        Imgproc.rectangle(input,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar(0, 255, 0), 4);
        return input;
    }
}

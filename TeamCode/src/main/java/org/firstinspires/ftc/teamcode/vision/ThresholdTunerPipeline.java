package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ThresholdTunerPipeline extends OpenCvPipeline {
    /*LAB best values
    double threshold1Low=0;
    double threshold2Low=138;
    double threshold3Low=135;
    double threshold1High=1000;
    double threshold2High=170;
    double threshold3High=165;

     */
    double threshold1Low=7;
    double threshold2Low=165;
    double threshold3Low=0;
    double threshold1High=13;
    double threshold2High=200;
    double threshold3High=1000;
    Telemetry t;
    public ThresholdTunerPipeline(Telemetry t){
        super();
        this.t = t;
    }
    public Mat processFrame(Mat input) {
        Mat yCrCbmat = new Mat();
        Imgproc.cvtColor(input,yCrCbmat,Imgproc.COLOR_RGB2HSV);
        for(int i = 0; i < yCrCbmat.cols();i++){
            for(int j = 0; j < yCrCbmat.rows();j++){
                Scalar currentPixel = Core.mean(yCrCbmat.submat(new Rect(i,j,1,1)));
                if(isInThreshold(currentPixel.val[0],threshold1Low,threshold1High)&&isInThreshold(currentPixel.val[1],threshold2Low,threshold2High)&& isInThreshold(currentPixel.val[2],threshold3Low,threshold3High)){
                    Imgproc.rectangle(yCrCbmat,new Rect(i,j,1,1),new Scalar(0,255,0));
                    t.addLine("areas within threshold detected");
                }
                else{
                    Imgproc.rectangle(yCrCbmat,new Rect(i,j,1,1),new Scalar(255,0,0));
                }
            }
        }
        return yCrCbmat;
    }
    public boolean isInThreshold(double number, double thresholdLow, double thresholdHigh){
        return (number > thresholdLow) && (number < thresholdHigh);
    }
}

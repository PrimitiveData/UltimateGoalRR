package org.firstinspires.ftc.teamcode.vision.BlobDetection;

import org.firstinspires.ftc.teamcode.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class BlobDetectorPipeline extends OpenCvPipeline {

    public int CbThresholdHigh=150; //Lower than threshold = is full yellow rect
    public int CbThresholdLow=135;
    public int distanceThreshold;
    public int percentFillThreshold;
    @Override
    public Mat processFrame(Mat input) {
        Mat YCrCbImg = new Mat();
        Imgproc.cvtColor(input,YCrCbImg,Imgproc.COLOR_RGB2YCrCb);
        ArrayList<Blob> blobs = new ArrayList<Blob>();
        int TOTAL_WIDTH = YCrCbImg.width();
        int TOTAL_HEIGHT = YCrCbImg.height();
        boolean[][] passesYCrCbThresholds = new boolean[TOTAL_WIDTH][TOTAL_HEIGHT];
        for(int i = 0; i < TOTAL_WIDTH; i++){
            for(int j = 0; j < TOTAL_HEIGHT;j++){
                double[] YCrCb = YCrCbImg.get(i,j);
                passesYCrCbThresholds[i][j] = passesThreshold(YCrCb[0], YCrCb[1], YCrCb[2]);
            }
        }
        for(int i = 0; i < TOTAL_WIDTH; i++){
            for(int j = 0; j < TOTAL_HEIGHT; j++){
                if(passesYCrCbThresholds[i][j]){
                    boolean isInExistingMat = false;
                    for(Blob b: blobs){
                        isInExistingMat = b.add(i,j,passesYCrCbThresholds,distanceThreshold,percentFillThreshold );
                    }
                    if(!isInExistingMat){
                        blobs.add(new Blob(i,j));
                    }
                }
            }
        }
        for(Blob b: blobs){
            Imgproc.rectangle(input, new Point(b.Xmin,b.Ymin), new Point(b.Xmax,b.Ymax), new Scalar(255,255,255),5);
        }
        return input;
    }

    public boolean passesThreshold(double Y, double Cr, double Cb){
        return Cb > CbThresholdLow && Cb < CbThresholdHigh;
    }
}

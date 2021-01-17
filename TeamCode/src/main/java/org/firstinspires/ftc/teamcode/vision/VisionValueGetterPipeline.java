package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;

public class VisionValueGetterPipeline extends OpenCvPipeline {
    double BLx;
    double BLy;
    double TRx;
    double TRy;
    public double[] highlightedBox = new double[]{0,0,0};
    double[] rangeValue1;
    double[] rangeValue2;
    double[] rangeValue3;
    public VisionValueGetterPipeline(double BLx, double BLy, double TRx, double TRy){
        super();
        this.BLx=BLx;
        this.BLy=BLy;
        this.TRx=TRx;
        this.TRy=TRy;
        rangeValue1 = new double[2];
        rangeValue2 = new double[2];
        rangeValue3 = new double[2];
    }
    public Mat processFrame(Mat input) {

        ArrayList<Double> allValue1;
        ArrayList<Double> allValue2;
        ArrayList<Double> allValue3;

        allValue1 = new ArrayList<Double>();
        allValue2 = new ArrayList<Double>();
        allValue3 = new ArrayList<Double>();
        Mat convertedMat = new Mat();
        Imgproc.cvtColor(input,convertedMat,Imgproc.COLOR_RGB2HSV);
        double[] highlightedBox1 = getAverageValues(convertedMat,(int)BLx,(int)BLy,(int)TRx,(int)TRy,allValue1,allValue2,allValue3);
                //Core.mean(convertedMat.submat(new Rect(new Point(BLx,BLy), new Point(TRx,TRy)))).val;
        if(highlightedBox1==null){
            highlightedBox=new double[]{0,0,0};
        }
        else {
            highlightedBox = highlightedBox1;
            rangeValue1 = sortAndGetCenterRange(0.9,allValue1);
            rangeValue2 = sortAndGetCenterRange(0.9,allValue2);
            rangeValue3 = sortAndGetCenterRange(0.9,allValue3);
        }
        Imgproc.rectangle(input,new Point(BLx,BLy), new Point(TRx,TRy),new Scalar(255,0,0));
        return input;
    }
    public double[] getAverageValues(Mat input, int botLeftCornerX, int botLeftCornerY, int topRightCornerX, int topRightCornerY, ArrayList<Double> arrayListToAddTo1, ArrayList<Double> arrayListToAddTo2, ArrayList<Double> arrayListToAddTo3){
        double value1Total=0;
        double value2Total=0;
        double value3Total=0;
        for(int i = botLeftCornerX; i < topRightCornerX; i++){
            for(int j = botLeftCornerY; j < topRightCornerY; j++){
                double[] values = input.get(j,i);
                if(values == null){
                }
                else{
                    arrayListToAddTo1.add(values[0]);
                    arrayListToAddTo2.add(values[1]);
                    arrayListToAddTo3.add(values[2]);
                    value1Total += values[0];
                    value2Total += values[1];
                    value3Total += values[2];
                }
            }
        }
        double totalArea = (topRightCornerX-botLeftCornerX)*(topRightCornerY-botLeftCornerY);
        return new double[]{value1Total/totalArea,value2Total/totalArea,value3Total/totalArea};
    }
    public double[] sortAndGetCenterRange(double centerPercent, ArrayList<Double> toSort){
        Collections.sort(toSort);
        double edgePercent = (1-centerPercent)/2;
        return new double[]{toSort.get((int) (edgePercent * toSort.size())), toSort.get((int) ((1-edgePercent) *toSort.size()))};
    }
}

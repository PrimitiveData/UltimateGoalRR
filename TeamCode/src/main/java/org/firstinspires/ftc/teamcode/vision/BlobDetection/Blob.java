package org.firstinspires.ftc.teamcode.vision.BlobDetection;

public class Blob {
    int Xmin;
    int Ymin;
    int Xmax;
    int Ymax;
    public Blob(int X, int Y){
        Xmax = X;
        Xmin = X;
        Ymax = Y;
        Ymin = Y;
    }
    public boolean isNear(int X, int Y, int distanceThreshold){
        return Math.hypot(X-(Xmin+Xmax)/2,Y-(Ymin+Ymax)/2) < distanceThreshold;
    }
    public boolean add(int X, int Y, boolean[][] passesThresholds, int distanceThreshold, int percentFillThreshold){
        if(!isNear(X,Y,distanceThreshold)){
            return false;
        }
        int[] XScanMinMax = getScanMinAndMax(X,Xmin,Xmax);
        int[] YScanMinMax = getScanMinAndMax(Y,Ymin,Ymax);
        int TotalXFills = 0;
        for(int i = XScanMinMax[0]; i < XScanMinMax[1]; i++){
            if(passesThresholds[i][Y]){
                TotalXFills++;
            }
        }
        double XPercentFill;
        if(XScanMinMax[0]==XScanMinMax[1]){
            XPercentFill = 100;
        }
        else {
            XPercentFill = TotalXFills * 1.0 / (XScanMinMax[0] - XScanMinMax[1]);
        }
        int TotalYFills = 0;
        for(int i = YScanMinMax[0]; i < YScanMinMax[1]; i++){
            if(passesThresholds[X][i]){
                TotalYFills++;
            }
        }
        double YPercentFill;
        if(YScanMinMax[0]==YScanMinMax[1]){
            YPercentFill = 100;
        }
        else {
            YPercentFill = TotalYFills * 1.0 / (YScanMinMax[0] - YScanMinMax[1]);
        }
        if(YPercentFill > percentFillThreshold && XPercentFill > percentFillThreshold){
            Xmin = Math.min(X, Xmin);
            Xmax = Math.max(X, Xmax);
            Ymin = Math.min(Y, Ymin);
            Ymax = Math.max(Y, Ymax);
            return true;
        }
        else{
            return false;
        }
    }
    public int[] getScanMinAndMax(int current, int min, int max){
        int ScanStart;// position from where we start our row scan;
        int ScanEnd;// position at which we end our row scan;
        if(current > max){
            ScanStart = min;
            ScanEnd = current;
            return new int[]{ScanStart,ScanEnd};
        }
        else if(current < min){
            ScanStart = current;
            ScanEnd = max;
            return new int[]{ScanStart,ScanEnd};
        }
        else{
            return new int[]{min,max};
        }
    }
}

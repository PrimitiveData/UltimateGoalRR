package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BlackBoxDetectorPipeline extends OpenCvPipeline {
    String TAG = "blackboxdetectorpipeline";
    Telemetry telemetry;
    public BlackBoxDetectorPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public Mat processFrame(Mat input){
       /* Imgproc.putText(input, );
        Imgproc.pu*/
       Mat yCrCbmat = new Mat();
       Imgproc.cvtColor(input,yCrCbmat,Imgproc.COLOR_RGB2GRAY);
        /*for(int i = 0; i < input.rows(); i++){
            for(int j = 0; j < input.cols(); j++){
                double[] RGB = input.get(i,j);
                RobotLog.dd(TAG,"R: " + RGB[0]+ ", G: " + RGB[1] + ", B: " + RGB[2]);
            }
        }*/
        double totalGreyScaleValue = 0;
        for(int i = 300; i < 341; i++){
            for (int j = 220; j < 261; j++){
                double[] RGB = yCrCbmat.get(j,i);
                double greyScaleValue = RGB[0];
                totalGreyScaleValue += greyScaleValue;
            }
        }
        double avgGreyScaleValue = totalGreyScaleValue / (40*40);
        telemetry.addData("avg greyscale: ", avgGreyScaleValue);
        telemetry.update();
        if(avgGreyScaleValue < 60){
            Imgproc.putText(input, "BLACK", new Point(300,270),Imgproc.FONT_HERSHEY_COMPLEX,100,new Scalar(0,255,0));
            telemetry.addLine("black");
        }
        Imgproc.rectangle(input, new Point(300,220), new Point(340,260), new Scalar(255,0,0),5);
        return input;
    }
}

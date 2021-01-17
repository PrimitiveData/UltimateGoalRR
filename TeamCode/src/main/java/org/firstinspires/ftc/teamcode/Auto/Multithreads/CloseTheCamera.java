package org.firstinspires.ftc.teamcode.Auto.Multithreads;

import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCamera;

public class CloseTheCamera extends Thread {
    OpenCvCamera webcam;
    public CloseTheCamera(OpenCvCamera webcam){
        this.webcam = webcam;
    }
    public void run(){
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}

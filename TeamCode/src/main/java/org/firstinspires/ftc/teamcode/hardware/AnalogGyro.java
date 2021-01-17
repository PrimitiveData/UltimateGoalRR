package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AnalogGyro {
    HardwareMap hardwareMap;
    double scaleFactor; //V/Deg/S
    double currentAngularPos;
    ElapsedTime time;
    double previousTime;
    boolean firstUpdate = true;
    public AnalogGyro(HardwareMap hardwareMap, ElapsedTime time){
        this.hardwareMap = hardwareMap;
        currentAngularPos = 0;
        this.time = time;
    }
    AnalogInput rateOut = hardwareMap.get(AnalogInput.class, "RateOut");
    AnalogInput SUMJ = hardwareMap.get(AnalogInput.class, "SUMJ");
    AnalogInput temp = hardwareMap.get(AnalogInput.class, "temp");
    public double getAngularVelocity(){
        return (rateOut.getVoltage()-SUMJ.getVoltage())/scaleFactor;
    }
    public void update(){
        double angularVelo = getAngularVelocity();
        double currentTime;
        if(firstUpdate){
            previousTime = time.milliseconds();
            currentTime = previousTime;
            firstUpdate = false;
        }
        else{
            currentTime = time.milliseconds();
        }
        currentAngularPos += angularVelo * (currentTime - previousTime) / 1000;
        previousTime = currentTime;
    }
    public double getAngle(){
        return currentAngularPos;
    }
}

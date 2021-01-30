package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AnalogGyro {
    HardwareMap hardwareMap;
    double scaleFactor = 0.00264; //V/Deg/S
    double currentAngularPos;
    double angularVelo;
    ElapsedTime time;
    double previousTime;
    boolean firstUpdate = true;
    AnalogInput rateOut;
    public double rateOutVoltage;
    AnalogInput temp;
    public AnalogGyro(HardwareMap hardwareMap, ElapsedTime time){
        this.hardwareMap = hardwareMap;
        currentAngularPos = 0;
        this.time = time;
        rateOut = hardwareMap.get(AnalogInput.class,"RateOut");
    }
    private double getAngularVelocity(){
        rateOutVoltage = getRateOutVoltage();
        if(rateOutVoltage == 1.6600000000000001 || rateOutVoltage == 1.661 || rateOutVoltage == 1.663 || rateOutVoltage == 1.659 ||
                rateOutVoltage == 1.6580000000000001||rateOutVoltage == 1.6640000000000001||rateOutVoltage == 1.6560000000000001){
            return 0;
        }
        return (rateOutVoltage-1.661319);
    }
    public void update(){
        angularVelo = getAngularVelocity();
        double currentTime;
        if(firstUpdate){
            previousTime = time.milliseconds();
            currentTime = previousTime;
            firstUpdate = false;
        }
        else{
            currentTime = time.milliseconds();
        }
        currentAngularPos += angularVelo/scaleFactor * (currentTime - previousTime) / 1000;
        previousTime = currentTime;
    }
    public double getAngle(){
        return currentAngularPos;
    }
    public double getAngularVelo(){
        return angularVelo;
    }
    public double getRateOutVoltage(){
        return rateOut.getVoltage();
    }
}

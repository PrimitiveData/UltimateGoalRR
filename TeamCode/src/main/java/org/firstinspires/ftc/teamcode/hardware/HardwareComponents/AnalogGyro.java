package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AnalogGyro {
    HardwareMap hardwareMap;
    double currentAngularPos;
    double angularVelo;
    double prevAngularVelo;
    ElapsedTime time;
    double previousTime;
    boolean firstUpdate = true;
    AnalogInput rateOut;
    public double rateOutVoltage;
    AnalogInput temp;
    final double yInterceptPositiveVelo = 15.01112822;
    final double slopePositiveVelo = -9.035492202;
    final double yInterceptNegativeVelo = 15.02728039;
    final double slopeNegativeVelo = -9.050097679;
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
        else if(rateOutVoltage > 1.6640000000000001) {
            return 360/357*360/357*1000*(rateOutVoltage * slopeNegativeVelo + yInterceptNegativeVelo);
        }
        else{
            return 360/357*360/357*1000*(rateOutVoltage * slopePositiveVelo + yInterceptPositiveVelo);
        }
    }
    public void update(){
        angularVelo = getAngularVelocity();
        double currentTime;
        if(firstUpdate){
            previousTime = time.milliseconds();
            currentTime = previousTime;
            firstUpdate = false;
            prevAngularVelo = angularVelo;
        }
        else{
            currentTime = time.milliseconds();
        }
        currentAngularPos += (angularVelo+prevAngularVelo)/2 * (currentTime - previousTime) / 1000;
        previousTime = currentTime;
        prevAngularVelo = angularVelo;
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

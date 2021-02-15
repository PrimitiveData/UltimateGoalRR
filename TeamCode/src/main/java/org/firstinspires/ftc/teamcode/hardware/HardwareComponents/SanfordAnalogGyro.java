package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SanfordAnalogGyro {
    HardwareMap hardwareMap;
    ElapsedTime time;
    AnalogInput out;
    public SanfordAnalogGyro(HardwareMap hardwareMap, ElapsedTime time){
        this.hardwareMap = hardwareMap;
        this.time = time;
        out = hardwareMap.get(AnalogInput.class,"RateOut");
    }
    public double getAngle(){
        return out.getVoltage() / 3.3 * 2*Math.PI;
    }

}

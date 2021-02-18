package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MathFunctions;

public class SanfordAnalogGyro {
    HardwareMap hardwareMap;
    AnalogInput out;
    double corectionCoeff = -360/358.0*7200/7319.98;
    boolean firstUpdateLoop;
    double prevAngle;
    double cumulativeAngle;
    public SanfordAnalogGyro(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        out = hardwareMap.get(AnalogInput.class,"SanfordAnalogGyroOutput");
        firstUpdateLoop = true;
        cumulativeAngle = 0;
    }
    public double getAngleRaw(){
        return out.getVoltage() / 3.3 *360/354* 2*Math.PI;
    }
    public void update(){
        if(firstUpdateLoop){
            prevAngle = getAngleRaw();
            firstUpdateLoop = false;
        }
        double currentAngle = getAngleRaw();
        cumulativeAngle += MathFunctions.keepAngleWithin180Degrees(currentAngle-prevAngle) * corectionCoeff;
        prevAngle = currentAngle;
    }
    public double getAngleCorrected(){
        return cumulativeAngle;
    }
}

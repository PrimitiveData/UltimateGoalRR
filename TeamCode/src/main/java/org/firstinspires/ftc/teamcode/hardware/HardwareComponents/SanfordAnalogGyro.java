package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MathFunctions;

public class SanfordAnalogGyro {
    HardwareMap hardwareMap;
    AnalogInput out;
    double corectionCoeff = -360/358.0*7200/7319.98 * 1.002087682672234 * 360/360.3990130404954;
    boolean firstUpdateLoop;
    double prevAngle;
    double cumulativeAngle;
    public SanfordAnalogGyro(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        out = hardwareMap.get(AnalogInput.class,"SanfordAnalogGyroOutput");
        firstUpdateLoop = true;
        cumulativeAngle = Math.PI;
    }
    public double getAngleRaw(){
        return out.getVoltage() / 3.3 *360/354* 360/360.22187981510024*2*Math.PI;
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
        return MathFunctions.keepAngleWithin180Degrees(cumulativeAngle);
    }
}

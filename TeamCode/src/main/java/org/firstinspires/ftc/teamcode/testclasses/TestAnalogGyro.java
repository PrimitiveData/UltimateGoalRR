package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.AnalogGyro;

import java.util.ArrayList;

@TeleOp(name="AnalogGyroTester", group="TeleOp")
public class TestAnalogGyro extends OpMode {
    AnalogGyro analogGyro;
    public ArrayList<Double> rateOutVoltages;
    public double rateOutVoltageIntegral;
    ElapsedTime time;
    double prevTime;
    double startTime;
    public void init(){
        analogGyro = new AnalogGyro(hardwareMap,new ElapsedTime());
        rateOutVoltages = new ArrayList<Double>();
        time = new ElapsedTime();
        rateOutVoltageIntegral = 0;
    }
    public void start(){
        prevTime = time.milliseconds();
        startTime = prevTime;
    }
    public void loop(){
        analogGyro.update();
        double currentTime = time.milliseconds();
        double deltaTime = currentTime - prevTime;
        prevTime = currentTime;
        telemetry.addData("analogGyroPosition",analogGyro.getAngle());
        telemetry.addData("RateOutVoltage",analogGyro.rateOutVoltage);
        if(rateOutVoltages.indexOf(analogGyro.rateOutVoltage)==-1){
            rateOutVoltages.add(analogGyro.rateOutVoltage);
        }
        rateOutVoltageIntegral += analogGyro.rateOutVoltage * deltaTime/1000;
        telemetry.addData("RateOutVoltageList",rateOutVoltages.toString());
        telemetry.addData("avgRateOutVoltage",rateOutVoltageIntegral/((currentTime-startTime)/1000));
    }
}

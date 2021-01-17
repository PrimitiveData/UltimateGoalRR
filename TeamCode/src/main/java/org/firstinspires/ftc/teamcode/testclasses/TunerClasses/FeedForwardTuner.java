package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="fftuner", group="TeleOp")
public class FeedForwardTuner extends LinearOpMode {
    Hardware hardware;
    String TAG = "feedfowardtuner";
    public void runOpMode(){
        hardware = new Hardware(hardwareMap,telemetry);
        waitForStart();
        double prevVelo = hardware.hub1Motors[0].motor.getVelocity(AngleUnit.RADIANS) *(Hardware.circumfrence/(2* Math.PI));
        FileWriter writer;
        try {
            writer = new FileWriter("//sdcard//FIRST//feedforwarddata.txt");
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        ElapsedTime time = new ElapsedTime();
        double startTime = time.milliseconds();
        double currentTime = startTime;
        double prevTime = startTime;
        double prevDist = hardware.localY;
        while(!isStopRequested() && currentTime < startTime + 6000){
            sleep(50);
            double timeDiff = currentTime - startTime;
            double i = timeDiff/6000;
            if(isStopRequested()){
                hardware.sixWheelDrive.goStraight(-1);
                sleep(100);
            }
            hardware.sixWheelDrive.goStraight(i);
            telemetry.addData("power",i);
            hardware.loop();
            currentTime = time.milliseconds();
            double velocity = hardware.localY* Hardware.circumfrence/ Hardware.ticks_per_rotation/((currentTime-prevTime)/1000);
            double accel = (velocity-prevVelo)/((currentTime-prevTime)/1000);
            prevTime = currentTime;
            prevVelo = velocity;
            RobotLog.dd(TAG, "Velo: " + velocity + ", Power: " + i + ", Accel: "+ accel+", Voltage: " + getBatteryVoltage() * i);
            try {
                writer.write("Velo: " + velocity + ", Power: " + i + ", Accel: "+ accel+", Voltage: " + getBatteryVoltage() * i+"\n");
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        try {
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}

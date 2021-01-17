package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="fftunergeneric", group="TeleOp")
public class FeedForwardTunerGeneric extends LinearOpMode {
    final double ZERO_TO_MAX_TIME = 120000;//milliseconds
    public void runOpMode(){
        DcMotor toTest = hardwareMap.get(DcMotor.class,"FFTuned");
        DcMotor toTest2 = hardwareMap.get(DcMotor.class,"FFTuned2");
        toTest.setDirection(DcMotor.Direction.FORWARD);
        toTest2.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        FileWriter writer;
        try {
            writer = new FileWriter("//sdcard//FIRST//feedforwarddatageneric.txt");
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        ElapsedTime time = new ElapsedTime();
        double startTime = time.milliseconds();
        double currentTime = startTime;
        double prevTime = startTime;
        double prevPosition = toTest.getCurrentPosition();
        double prevVelo = 0;
        while(!isStopRequested() && currentTime < startTime + ZERO_TO_MAX_TIME){
            double powerToSet = (currentTime-startTime)/ZERO_TO_MAX_TIME;
            toTest.setPower(-powerToSet);
            toTest2.setPower(-powerToSet);
            sleep(50);
            double currentPosition = toTest.getCurrentPosition();
            currentTime = time.milliseconds();
            double velocity = (currentPosition - prevPosition)/((currentTime-prevTime)/1000);
            prevPosition = currentPosition;
            double accel = (velocity-prevVelo)/((currentTime-prevTime)/1000);
            prevVelo = velocity;
            prevTime = currentTime;
            try {
                writer.write("Velo: " + velocity + ", Accel: "+ accel+", Voltage: " + getBatteryVoltage() * powerToSet+"\n");
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

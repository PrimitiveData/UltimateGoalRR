package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
@Disabled
public class FeedForwardTunerGeneric extends LinearOpMode {
    final double ZERO_TO_MAX_TIME = 180000;//milliseconds
    public void runOpMode(){
        DcMotorEx toTest = hardwareMap.get(DcMotorEx.class,"FFTuned");
        DcMotorEx toTest2 = hardwareMap.get(DcMotorEx.class,"FFTuned2");
        toTest.setDirection(DcMotor.Direction.REVERSE);
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
        while(!isStopRequested() && time.milliseconds()-startTime < ZERO_TO_MAX_TIME){
            double powerToSet = (time.milliseconds()-startTime)/ZERO_TO_MAX_TIME;
            toTest.setPower(-powerToSet);
            toTest2.setPower(-powerToSet);
            sleep(50);
            double velocity = toTest.getVelocity();
            if(velocity == 0){
                velocity = toTest2.getVelocity();
            }
            try {
                writer.write("Velo: " + velocity + ", Accel: "+ 0+", Voltage: " + getBatteryVoltage() * powerToSet+"\n");
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

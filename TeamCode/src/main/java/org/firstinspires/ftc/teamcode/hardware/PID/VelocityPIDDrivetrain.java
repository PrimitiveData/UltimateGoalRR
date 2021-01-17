package org.firstinspires.ftc.teamcode.hardware.PID;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.io.FileWriter;
import java.io.IOException;

public class VelocityPIDDrivetrain extends GenericPID {
    public double kV;
    double kStatic;
    double kA;
    double kDecel;
    public FileWriter writer;
    double prevVelo;
    double accel;
    double prevTimeAccel;
    double kVAngularVelo;
    double targetAngularVelo;
    double kStaticAngularVelo;
    double kAAngularAccel;
    double angularAccel;
    public VelocityPIDDrivetrain(double kP, double kI, double kD, double kV, double kStatic, double kA, double kDecel, double kVAngularVelo, double kStaticAngularVelo, double kAAngularAccel, ElapsedTime time, String outputFileName) {
        super(kP, kI, kD, time);
        try {
            writer = new FileWriter(outputFileName);
        }
        catch(IOException e){
            return;
        }
        this.kV = kV;
        this.kStatic = kStatic;
        this.kA = kA;
        this.kDecel = kDecel;
        desiredState = 0;
        prevVelo = 0;
        accel = 0;
        this.kVAngularVelo = kVAngularVelo;
        this.kStaticAngularVelo = kStaticAngularVelo;
        this.kAAngularAccel = kAAngularAccel;
    }
    public synchronized void setState(double targetVelocity, double accel){
        if(firstSetStateLoop){
            firstSetStateLoop=false;
            prevTimeAccel = time.milliseconds();
        }
        this.desiredState = targetVelocity;
        double currentTime = time.milliseconds();
        this.accel = accel;
        prevTimeAccel = currentTime;
        prevVelo = targetVelocity;
        targetAngularVelo = 0;
    }
    public synchronized void setState(double targetVelocity, double accel, double targetAngularVelo, double angularAccel){
        if(firstSetStateLoop){
            firstSetStateLoop=false;
            prevTimeAccel = time.milliseconds();
        }
        this.desiredState = targetVelocity;
        double currentTime = time.milliseconds();
        this.accel = accel;
        prevTimeAccel = currentTime;
        prevVelo = targetVelocity;
        this.targetAngularVelo = targetAngularVelo;
        this.angularAccel = angularAccel;
    }
    public synchronized double updateCurrentStateAndGetOutput(double currentVelocity){
        double currentTime = time.milliseconds();
        if(firstGetOutputLoop){
            firstGetOutputLoop = false;
            prevTime = currentTime;
            startTime = prevTime;
            prevError = 0;
        }
        double deltaTime = (currentTime - prevTime)/1000;
        prevTime = currentTime;
        double error = desiredState - currentVelocity;
        double deltaError = error-prevError;
        prevError = error;
        integral+=error*deltaTime;
        double derivative = 0;
        if(deltaTime != 0) {
             derivative = deltaError / deltaTime;
        }
        double batteryVoltage = getBatteryVoltage();
        RobotLog.dd("FFPID","error: "+error + ", AngularAccel: "+angularAccel);
        double toReturn;
        if(desiredState < 0){
            if((desiredState >= 0 && accel > 0) ||(desiredState <= 0 && accel < 0)) {
                toReturn = error * kP + integral * kI + derivative * kD - kStatic / batteryVoltage + (desiredState * kV) / batteryVoltage + (accel * kA) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
            else if((desiredState < 0 && accel > 0)||(desiredState > 0 && accel < 0)){
                toReturn = error * kP + integral * kI + derivative * kD - kStatic / batteryVoltage + (desiredState * kV) / batteryVoltage + (accel * kDecel) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
            else{
                toReturn = error * kP + integral * kI + derivative * kD - kStatic / batteryVoltage + (desiredState * kV) / batteryVoltage + (accel * kA) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
        }
        else{
            if((desiredState >= 0 && accel > 0) ||(desiredState <= 0 && accel < 0)) {
                toReturn = error * kP + integral * kI + derivative * kD + kStatic / batteryVoltage + (desiredState * kV) / batteryVoltage + (accel * kA) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
            else if((desiredState < 0 && accel > 0)||(desiredState > 0 && accel < 0)){
                toReturn = error * kP + integral * kI + derivative * kD + kStatic / batteryVoltage + (desiredState * kV) / batteryVoltage + (accel * kDecel) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
            else{
                toReturn = error * kP + integral * kI + derivative * kD + kStatic / batteryVoltage + (desiredState * kV) / batteryVoltage + (accel * kA) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
        }
        try {
            writer.write("Time: " + (currentTime-startTime)/1000+", RequestedV: " + desiredState + ", Velo: " + currentVelocity+", Power: "+toReturn+ ", "+ ", FFonlyVoltage: " + (desiredState*kV+accel*kA)+ ", Accel: " + accel+ "\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
        return toReturn;
    }
    public static double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : Hardware.getInstance().hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}

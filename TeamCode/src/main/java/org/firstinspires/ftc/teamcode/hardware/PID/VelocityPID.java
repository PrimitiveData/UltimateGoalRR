package org.firstinspires.ftc.teamcode.hardware.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

public class VelocityPID extends GenericPID {
    public double kV;
    double kStatic;
    double kA;
    public Writer writer;
    public VelocityPID(double kP, double kI, double kD, double kV, double kStatic, double kA, ElapsedTime time, String outputFileName) {
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
        desiredState = 0;
    }
    public synchronized void setState(double targetVelocity){
        this.desiredState = targetVelocity;
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
        if(integralAntiWindupActive && shouldIntegralBeZeroed(error)){
            clearI();
        }
        double derivative = 0;
        if(deltaTime != 0) {
            derivative = deltaError / deltaTime;
        }

        try {
            writer.write("Time: " + (currentTime-startTime)/1000+", RequestedV: " + desiredState + ", Velo: " + currentVelocity+"\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
        if(desiredState > 0) {
            return error * kP + integral * kI + derivative * kD + kStatic + (desiredState * kV);
        }
        else{
            return error * kP + integral * kI + derivative * kD - kStatic + (desiredState * kV);
        }
    }
}

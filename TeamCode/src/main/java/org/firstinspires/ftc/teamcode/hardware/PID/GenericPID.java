package org.firstinspires.ftc.teamcode.hardware.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

public class GenericPID {
    public double desiredState;
    public double currentState;
    public double kP;
    public double kI;
    public double kD;
    public ElapsedTime time;
    public double integral;
    public boolean firstSetStateLoop;
    public boolean firstGetOutputLoop;
    public double prevTime;
    public double prevError;
    public double startTime;
    public boolean integralAntiWindupActive;
    public GenericPID(double kP, double kI, double kD, ElapsedTime time){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.time = time;
        integral = 0;
        firstGetOutputLoop = true;
        firstSetStateLoop = true;
        integralAntiWindupActive = false;
    }
    public void clearI(){
        integral = 0;
    }
    public void setState(double desiredState){
        this.desiredState = desiredState;
    }
    public double updateCurrentStateAndGetOutput(double currentState){
        double currentTime = time.milliseconds();
        if(firstGetOutputLoop){
            firstGetOutputLoop = false;
            prevTime = currentTime;
            startTime = prevTime;
            prevError = 0;
        }

        double deltaTime = (currentTime - prevTime)/1000;
        prevTime = currentTime;
        double error = desiredState - currentState;
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
        this.currentState = currentState;
        return kP * error + kI * integral + kD * derivative;
    }

    public boolean shouldIntegralBeZeroed(double error){
        return false;
    }


}

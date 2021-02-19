package org.firstinspires.ftc.teamcode.hardware.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.CharArrayReader;
import java.io.IOException;

public class ShooterPID extends VelocityPID {
    public double disableIntegralThreshold;
    public boolean speedyRecoveryOn = true;
    public boolean spinningUp;
    public ElapsedTime clearIAtBeginningTimer;
    public ShooterPID(double kP, double kI, double kD, double kV, double kStatic, double kA, double disableIntegralThreshold, ElapsedTime time, String outputFileName) {
        super(kP, kI, kD, kV, kStatic, kA, time, outputFileName);
        this.disableIntegralThreshold = disableIntegralThreshold;
        spinningUp = true;
        clearIAtBeginningTimer = new ElapsedTime();

    }
    @Override
    public synchronized double updateCurrentStateAndGetOutput(double currentVelocity){
        currentState = currentVelocity;
        double currentTime = time.milliseconds();
        if(firstGetOutputLoop){
            firstGetOutputLoop = false;
            prevTime = currentTime;
            startTime = prevTime;
            prevError = 0;
            spinningUp = true;
            clearIAtBeginningTimer.reset();
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
        double voltage = VelocityPIDDrivetrain.getBatteryVoltage();
        /*if(currentVelocity > desiredState+75 && speedyRecoveryOn){
            return -1;
        }*/
        if(clearIAtBeginningTimer.milliseconds() < 3000){
            clearI();
        }

        if(spinningUp){
            if(desiredState < 0 && currentVelocity < desiredState + 350){
                spinningUp = false;
            }
            else if(desiredState > 0 && currentVelocity > desiredState - 350){
                spinningUp = false;
            }
            else if(desiredState == 0){
                spinningUp = false;
            }

            if(desiredState < 0){
                return  -1;
            }
            else if(desiredState > 0){
                return 1;
            }
            else{
                return 0;
            }
        }
        else if(desiredState == 0){
            return 0;
        }
        else if(desiredState > 0) {
            return (error * kP + integral * kI + derivative * kD + kStatic + (desiredState * kV))/voltage;
        }
        else {
            return (error * kP + integral * kI + derivative * kD - kStatic + (desiredState * kV))/voltage;
        }
    }
    @Override
    public boolean shouldIntegralBeZeroed(double error){
        return Math.abs(error)>disableIntegralThreshold;
    }
    @Override
    public void setState(double desiredState){
        if(desiredState == 0){
            firstGetOutputLoop = true;
        }
        else{
            firstGetOutputLoop = false;
        }
        this.desiredState = desiredState;
    }
}

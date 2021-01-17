package org.firstinspires.ftc.teamcode.hardware.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDwithBasePower extends TurretPID {
    public double kStatic;
    public double leewayDistance;
    public PIDwithBasePower(double kP, double kI, double kD, double kStatic, double leewayDistance, double disableIntegralThreshold, ElapsedTime time) {
        super(kP, kI, kD, disableIntegralThreshold, time);
        this.kStatic = kStatic;
        this.leewayDistance = leewayDistance;
    }
    @Override
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
        if(error > leewayDistance) {
            return kP * error + kI * integral + kD * derivative + kStatic;
        }
        else if (error < -leewayDistance){
            return kP * error + kI * integral + kD * derivative - kStatic;
        }
        else{
            return  0;
        }
    }
}

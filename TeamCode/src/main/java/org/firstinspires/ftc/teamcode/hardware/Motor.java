package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Motor {
    public boolean writeVelocityRequested;
    public boolean writePowerRequested;
    public boolean setTargetPosRequested;
    public boolean readRequested;
    public int targetPosition;
    public double power;
    public double velocity;
    public int currentPosition;
    public double currentVelocity;
    public DcMotorEx motor;
    public Motor(DcMotorEx motor){
        writeVelocityRequested = false;
        writePowerRequested = false;
        setTargetPosRequested = false;
        readRequested = false;
        targetPosition = 0;
        power = 0;
        velocity = 0;
        currentPosition = 0;
        this.motor = motor;
    }
    public void setPower(double power){
        writePowerRequested = true;
        this.power = power;
    }
    public void setVelocity(double velocity){
        writeVelocityRequested = true;
        this.velocity = velocity;
    }
    public void setTargetPosition(int targetPos){
        setTargetPosRequested = true;
        targetPosition = targetPos;
    }
    public int getCurrentPosition(){
        return  currentPosition;
    }
    public double getVelocity(){
        return currentVelocity;
    }
}

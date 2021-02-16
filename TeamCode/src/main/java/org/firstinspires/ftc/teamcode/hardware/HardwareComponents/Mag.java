package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Mag {
    public RegServo magServo;
    RegServo ringPusher;
    HardwareMecanum hardware;
    double ringPusherResting=0.8495;
    double ringPusherPushedIn=0.6684;
    double magCollectRingPosition=0.1706;
    double magDropRingPosition=0.619;
    double magRotationCollectPosition = 0.488;
    public State currentState;
    public Mag(RegServo magServo, RegServo ringPusher, HardwareMecanum hardware){
        this.magServo = magServo;
        this.ringPusher = ringPusher;
        this.hardware = hardware;
        currentState = State.COLLECT;
    }
    public void collectRings(){
        hardware.turret.magShootingState = false;
        hardware.turret.setMagAngle(magRotationCollectPosition);
        currentState = State.COLLECT;
        magServo.setPosition(magCollectRingPosition);
    }
    public void dropRings(){
        hardware.turret.magShootingState = true;
        currentState = State.DROP;
        magServo.setPosition(magDropRingPosition);
    }
    public void toggleStates(){
        if(currentState == State.DROP){
            currentState = State.COLLECT;
        }else if(currentState == State.COLLECT){
            currentState = State.DROP;
        }
    }
    public enum State{
        DROP,
        COLLECT
    }
    public void pushInRings(){
        ringPusher.setPosition(ringPusherPushedIn);
    }
    public void setRingPusherResting(){
        ringPusher.setPosition(ringPusherResting);
    }
}

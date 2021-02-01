package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Mag {
    public RegServo magServo;
    RegServo ringPusher;
    double ringPusherResting=0.38589; // must be tuned
    double ringPusherPushedIn=0.1425627; // must be tuned
    double magCollectRingPosition=0.045;
    double magDropRingPosition=0.39217;
    public State currentState;
    public Mag(RegServo magServo, RegServo ringPusher){
        this.magServo = magServo;
        this.ringPusher = ringPusher;
        currentState = State.COLLECT;
    }
    public void collectRings(){
        currentState = State.COLLECT;
        magServo.setPosition(magCollectRingPosition);
    }
    public void dropRings(){
        currentState = State.DROP;
        magServo.setPosition(magDropRingPosition);
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

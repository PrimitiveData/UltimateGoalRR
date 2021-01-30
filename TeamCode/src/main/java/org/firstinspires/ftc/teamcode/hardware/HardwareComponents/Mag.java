package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Mag {
    public RegServo magServo;
    RegServo ringPusher;
    double ringPusherResting; // must be tuned
    double ringPusherPushedIn; // must be tuned
    double magCollectRingPosition;
    double magDropRingPosition;
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

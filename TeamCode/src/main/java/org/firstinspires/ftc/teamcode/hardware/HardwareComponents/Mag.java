package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Mag {
    public RegServo magServo;
    RegServo ringPusher;
    HardwareMecanum hardware;
    double ringPusherResting=0.38589; // must be tuned
    double ringPusherPushedIn=0.1425627; // must be tuned
    double magCollectRingPosition=0.045;
    double magDropRingPosition=0.39217;
    double magRotationCollectPosition = 0.48;
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

package org.firstinspires.ftc.teamcode.Teleop.Multithreads;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Teleop.UltimateGoalTeleop;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;

import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

public class MagFlickerController extends Thread {
    public Hardware hardware;
    UltimateGoalTeleop parentOP;
    boolean shootRingRequested;
    boolean shootAllRingsRequested;
    boolean firstButtonPress = false;
    int numButtonPresses;
    String TAG = "MagFlickerController";
    public Writer writer;
    public MagFlickerController(Hardware hardware, UltimateGoalTeleop parentOP){
        this.hardware = hardware;
        this.parentOP = parentOP;
        shootRingRequested = false;
        shootAllRingsRequested = false;
        firstButtonPress = true;
        numButtonPresses = 0;
        try {
            writer = new FileWriter("//sdcard//FIRST//MagFlickerControllerData.txt");
        }
        catch(IOException e){
            return;
        }
    }
    public void sleeep(double milliseconds){
        double startTime = hardware.time.milliseconds();
        while(hardware.time.milliseconds() < startTime + milliseconds && !parentOP.teleopStopped){
            try{
                Thread.sleep(10);
            }catch(InterruptedException e){

            }
        }
    }
    public void run(){
        while(!parentOP.teleopStopped){
            if(shootAllRingsRequested){
                hardware.mag.currentState = Mag.State.COLLECT;
                parentOP.currentlyIncrementingMagDuringShooting = true;
                for(int i = 0; i < 3; i++){
                    hardware.mag.updateStateAndSetPosition();
                    sleeep(250);
                    hardware.mag.pushInRings();
                    sleeep(100);
                    hardware.mag.setRingPusherResting();
                    sleeep(75);
                    hardware.shooter.setRampPosition(hardware.shooter.rampPostion+=0.0125);
                }
                parentOP.currentlyIncrementingMagDuringShooting = false;
                shootAllRingsRequested=false;
                hardware.mag.updateStateAndSetPosition();
            }
            if(shootRingRequested){
                numButtonPresses++;
                RobotLog.dd(TAG,numButtonPresses+" press checkpoint 1: " + hardware.mag.currentState.toString());
                try {
                    writer.write(numButtonPresses+" press checkpoint 1: " + hardware.mag.currentState.toString());
                } catch (IOException e) {
                    e.printStackTrace();
                }
                hardware.mag.updateStateAndSetPosition();
                RobotLog.dd(TAG,numButtonPresses+" press checkpoint 2: " + hardware.mag.currentState.toString());
                try {
                    writer.write(numButtonPresses+" press checkpoint 2: " + hardware.mag.currentState.toString());
                } catch (IOException e) {
                    e.printStackTrace();
                }
                if(firstButtonPress){
                    hardware.mag.currentState = Mag.State.TOP;
                    firstButtonPress = false;
                }
                RobotLog.dd(TAG,numButtonPresses+" press checkpoint 3: " + hardware.mag.currentState.toString());
                try {
                    writer.write(numButtonPresses+" press checkpoint 3: " + hardware.mag.currentState.toString());
                } catch (IOException e) {
                    e.printStackTrace();
                }
                if(hardware.mag.currentState == Mag.State.COLLECT){
                    hardware.mag.setRingPusherResting();
                    RobotLog.dd(TAG,numButtonPresses+" press checkpoint 4: " + hardware.mag.currentState.toString());
                    try {
                        writer.write(numButtonPresses+" press checkpoint 4: " + hardware.mag.currentState.toString());
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
                else{
                    sleeep(225);
                    hardware.shooter.shooterVeloPID.speedyRecoveryOn = false;
                    hardware.mag.pushInRings();
                    RobotLog.dd(TAG,numButtonPresses+" press checkpoint 5 time: " + hardware.time.milliseconds());
                    try {
                        writer.write(numButtonPresses+" press checkpoint 5 time: " + hardware.time.milliseconds());
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    sleeep(175);
                    hardware.shooter.shooterVeloPID.speedyRecoveryOn = true;
                    hardware.mag.setRingPusherResting();
                    RobotLog.dd(TAG,numButtonPresses+" press checkpoint 6 time: " + hardware.time.milliseconds());
                    try {
                        writer.write(numButtonPresses+" press checkpoint 6 time: " + hardware.time.milliseconds());
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    sleeep(175);
                    if(hardware.mag.currentState == Mag.State.BOTTOM){
                        hardware.mag.updateStateAndSetPosition();
                    }
                }
                shootRingRequested = false;
            }
        }
    }
    public void updateMagStateAndSetPosition(){
        shootRingRequested = true;
    }
    public void updateMagStateAndSetPositionAndShootAllRings(){
        shootAllRingsRequested = true;
    }
}

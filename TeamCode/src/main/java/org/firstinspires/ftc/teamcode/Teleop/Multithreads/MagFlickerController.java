package org.firstinspires.ftc.teamcode.Teleop.Multithreads;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Teleop.UltimateGoalTeleop;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

public class MagFlickerController extends Thread {
    public HardwareMecanum hardware;
    UltimateGoalTeleop parentOP;
    ElapsedTime time;
    boolean shootAllRingsRequested;
    boolean firstButtonPress = false;
    int numButtonPresses;
    String TAG = "MagFlickerController";
    public Writer writer;
    public MagFlickerController(HardwareMecanum hardware, UltimateGoalTeleop parentOP){
        this.hardware = hardware;
        this.parentOP = parentOP;
        shootAllRingsRequested = false;
        firstButtonPress = true;
        numButtonPresses = 0;
        time = new ElapsedTime();
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
                if(hardware.mag.currentState == Mag.State.COLLECT) {
                    hardware.mag.dropRings();
                    sleeep(500);//tune timeout
                }
                for(int i = 0; i < 3; i++){
                    hardware.mag.pushInRings();
                    sleeep(200);// tune time
                    hardware.mag.setRingPusherResting();
                    sleeep(100);// tune time
                }
                hardware.mag.collectRings();
                shootAllRingsRequested = false;
            }
        }
        /*if(hardware.mag.currentState == Mag.State.COLLECT){
            if(time.milliseconds() % 1000 < 500)
                hardware.turret.setMagAngle(hardware.mag.magRotationCollectPosition + 0.019);
            else if(time.milliseconds() % 1000 >= 500)
                hardware.turret.setMagAngle(hardware.mag.magRotationCollectPosition - 0.019);
        }*/
    }
    public void shootAllRings(){
        shootAllRingsRequested = true;
    }
}

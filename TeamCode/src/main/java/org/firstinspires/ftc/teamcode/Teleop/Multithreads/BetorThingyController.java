package org.firstinspires.ftc.teamcode.Teleop.Multithreads;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleop.UltimateGoalTeleop;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.RegServo;

import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

public class BetorThingyController extends Thread {
    public HardwareMecanum hardware;
    UltimateGoalTeleop parentOP;

    boolean betorThingyFlickRequested;

    boolean isLeft;

    public BetorThingyController(HardwareMecanum hardware, UltimateGoalTeleop parentOP, boolean isLeft){
        this.hardware = hardware;
        this.parentOP = parentOP;
        betorThingyFlickRequested = false;
        this.isLeft = isLeft;
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

    public void flickBetorThingy(){
        betorThingyFlickRequested = true;
    }

    public void run(){
        while(!parentOP.teleopStopped){
            if(betorThingyFlickRequested){
                if(isLeft) {
                    hardware.intake.leftBetorThingyIn();
                    sleeep(250);
                    hardware.intake.leftBetorThingyOut();
                }else{
                    hardware.intake.rightBetorThingyIn();
                    sleeep(250);
                    hardware.intake.rightBetorThingyOut();
                }
                betorThingyFlickRequested = false;
            }
        }
    }
}

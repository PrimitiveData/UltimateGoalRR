package org.firstinspires.ftc.teamcode.Teleop.Multithreads;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
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
    boolean shootPowershotSequenceRequested;
    boolean firstButtonPress = false;
    int numButtonPresses;
    String TAG = "MagFlickerController";
    public Writer writer;
    public MagFlickerController(HardwareMecanum hardware, UltimateGoalTeleop parentOP){
        this.hardware = hardware;
        this.parentOP = parentOP;
        shootAllRingsRequested = false;
        shootPowershotSequenceRequested = false;
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
            if(parentOP.startMagRotation){
                hardware.turret.setMagAngle(0.8);
                sleeep(200);
                parentOP.startMagRotation = false;
                hardware.mag.dropRings();
            }
            if(shootAllRingsRequested){
                if(hardware.mag.currentState == Mag.State.COLLECT) {
                    hardware.turret.setMagAngle(0.5);
                    sleeep(100);
                    hardware.mag.dropRings();
                    sleeep(450);//tune timeout
                }
                for(int i = 0; i < 3; i++){
                    parentOP.currentlyIncrementingMagDuringShooting = true;
                    if(-hardware.shooter.shooterMotor1.getVelocity() >= 1325) {
                        hardware.mag.pushInRingsThreadBypass();
                        sleeep(100);
                        hardware.mag.setRingPusherRestingThreadBypass();
                        sleeep(100);
                    }
                    else
                        i--;
                }
                parentOP.currentlyIncrementingMagDuringShooting = false;
                hardware.mag.collectRings();
                shootAllRingsRequested = false;
            }
            if(shootPowershotSequenceRequested){
                if(hardware.mag.currentState == Mag.State.COLLECT){
                    hardware.mag.dropRings();
                    sleeep(500);
                }
                hardware.mag.dropRings();
                for(int i = 0; i < 3; i++){
                    hardware.mag.pushInRings();
                    sleeep(200);// tune time
                    hardware.mag.setRingPusherResting();
                    FieldConstants.highGoalPosition[1] -= 7.5;
                    sleeep(250);
                }
                FieldConstants.highGoalPosition[0] = FieldConstants.highGoalPositionBackUp[0];
                FieldConstants.highGoalPosition[1] = FieldConstants.highGoalPositionBackUp[1];
                shootPowershotSequenceRequested = false;
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
    public void shootPowershotAllRings(){
        shootPowershotSequenceRequested = true;
    }
}

package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Teleop.UltimateGoalTeleop;

public class HardwareThreadInterface extends Thread {
    public Hardware hardware;
    LinearOpMode parentOP;
    public HardwareMecanum hardwareMecanum;
    UltimateGoalTeleop teleop;
    public boolean stopLooping;
    public HardwareThreadInterface(Hardware hardware, LinearOpMode parentOP){
        this.hardware = hardware;
        this.parentOP = parentOP;
        stopLooping = false;
    }
    public HardwareThreadInterface(Hardware hardware, UltimateGoalTeleop teleop){
        this.hardware = hardware;
        this.teleop = teleop;
        stopLooping = false;
    }
    public HardwareThreadInterface(HardwareMecanum hardware, LinearOpMode parentOP){
        this.hardwareMecanum = hardware;
        this.parentOP = parentOP;
        stopLooping = false;
    }
    public void run(){
        if(parentOP != null) {
            while (!parentOP.isStopRequested()&&!stopLooping) {
                if (hardware != null) {
                    hardware.loop();
                } else {
                    hardwareMecanum.loop();
                }
                if (isInterrupted()) {
                    return;
                }
            }
        }
        if(teleop != null){
            while(!teleop.teleopStopped && !stopLooping){
                if(hardware != null){
                    hardware.loop();
                }else{
                    hardwareMecanum.loop();
                }
            }
        }
    }
}

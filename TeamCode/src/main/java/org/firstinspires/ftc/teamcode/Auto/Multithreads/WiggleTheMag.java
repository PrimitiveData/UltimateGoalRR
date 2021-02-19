package org.firstinspires.ftc.teamcode.Auto.Multithreads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

public class WiggleTheMag extends Thread{
    HardwareMecanum hardware;
    public boolean stopRequested = false;
    LinearOpMode parentOP;
    public WiggleTheMag(HardwareMecanum hardware, LinearOpMode parentOP){
        this.hardware = hardware;
        this.parentOP = parentOP;
    }
    public void run(){
        ElapsedTime time = hardware.time;
        while(!stopRequested&&!parentOP.isStopRequested()){
            if(!hardware.turret.magShootingState) {
                if (time.milliseconds() % 1500 >= 1000)
                    hardware.turret.setMagAngle(hardware.mag.magRotationCollectPosition + 0.017);
                else if (time.milliseconds() % 1500 >= 500)
                    hardware.turret.setMagAngle(hardware.mag.magRotationCollectPosition);
                else
                    hardware.turret.setMagAngle(hardware.mag.magRotationCollectPosition - 0.017);
            }
        }
    }
}

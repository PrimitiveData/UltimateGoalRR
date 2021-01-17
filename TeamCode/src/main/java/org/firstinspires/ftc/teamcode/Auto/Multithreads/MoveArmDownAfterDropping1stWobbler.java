package org.firstinspires.ftc.teamcode.Auto.Multithreads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

public class MoveArmDownAfterDropping1stWobbler extends Thread {
    Hardware hardware;
    LinearOpMode parentOP;
    public MoveArmDownAfterDropping1stWobbler(Hardware hardware, LinearOpMode parentOP){
        this.hardware = hardware;
        this.parentOP = parentOP;
    }
    public void run(){
        while(hardware.angle > -Math.toRadians(150) && !parentOP.isStopRequested()){
            parentOP.sleep(10);
        }
        hardware.wobbler.moveArmToGrabPos();
    }
}

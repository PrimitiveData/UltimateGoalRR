package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.hardware.PID.PIDwithBasePower;

public class MecanumDrive {
    public Motor LF;
    public Motor LB;
    public Motor RF;
    public Motor RB;
    HardwareMecanum hardware;
    public PIDwithBasePower turretPID;
    public MecanumDrive (Motor LF, Motor LB, Motor RF, Motor RB){
        this.LF = LF;
        this.LB = LB;
        this.RF = RF;
        this.RB = RB;
        turretPID = new PIDwithBasePower(0.87,0.7,0,0.15, Math.toRadians(0.5), Math.toRadians(20), hardware.time);
    }
    public void setPowers(double movementX, double movementY, double turnMagnitude){
        double LFpower = movementY-turnMagnitude-movementX;
        double LBpower = movementY-turnMagnitude+movementX;
        double RFpower = movementY+turnMagnitude+movementX;
        double RBpower = movementY+turnMagnitude-movementX;
        double largestPower = Math.abs(LFpower);
        if(Math.abs(LBpower) > Math.abs(LFpower))
            largestPower = Math.abs(LBpower);
        if(Math.abs(RFpower) > Math.abs(LBpower))
            largestPower = Math.abs(RFpower);
        if(Math.abs(RBpower) > Math.abs(RFpower)){
            largestPower = Math.abs(RBpower);
        }
        if(largestPower>1) {
            LFpower = LFpower / largestPower;
            LBpower = LBpower / largestPower;
            RFpower = RFpower / largestPower;
            RBpower = RBpower / largestPower;
        }
        LF.setPower(LFpower);
        LB.setPower(LBpower);
        RF.setPower(RFpower);
        RB.setPower(RBpower);
    }
    public void setPowersTeleop(double movementX, double movementY, double turnMagnitude){
        HardwareMecanum.telemetry.addData("movementY: ",movementY);
        HardwareMecanum.telemetry.update();
        double LFpower = movementY-turnMagnitude-movementX;
        double LBpower = movementY-turnMagnitude+movementX;
        double RFpower = movementY+turnMagnitude+movementX;
        double RBpower = movementY+turnMagnitude-movementX;
        /*double largestPower = Math.abs(LFpower);
        if(Math.abs(LBpower) > Math.abs(LFpower))
            largestPower = Math.abs(LBpower);
        if(Math.abs(RFpower) > Math.abs(LBpower))
            largestPower = Math.abs(RFpower);
        if(Math.abs(RBpower) > Math.abs(RFpower)){
            largestPower = Math.abs(RBpower);
        }
        if(largestPower > 1) {
            LFpower = LFpower / largestPower;
            LBpower = LBpower / largestPower;
            RFpower = RFpower / largestPower;
            RBpower = RBpower / largestPower;
        }*/
        LF.motor.setPower(LFpower);
        LB.motor.setPower(LBpower);
        RF.motor.setPower(RFpower);
        RB.motor.setPower(RBpower);
    }
}

package org.firstinspires.ftc.teamcode.Teleop.Multithreads;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.Teleop.UltimateGoalTeleop;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;

public class ShootPowershot extends Thread {
    Hardware hardware;
    UltimateGoalTeleop parentOP;
    Telemetry telemetry;
    public ShootPowershot(Hardware hardware, UltimateGoalTeleop parentOP, Telemetry telemetry){
        this.hardware = hardware;
        this.parentOP = parentOP;
        this.telemetry = telemetry;
    }
    public void shootPowershot(Hardware hardware) {
        hardware.mag.pushInRings();
        sleeep(175);
        hardware.mag.setRingPusherResting();
        sleeep(200);
        hardware.mag.updateStateAndSetPosition();
    }
    public void sleeep(double milliseconds){
        double startTime = hardware.time.milliseconds();
        while(hardware.time.milliseconds() < startTime + milliseconds && !parentOP.teleopStopped){
            try{
                Thread.sleep(10);
            }catch(InterruptedException e){

            }
            parentOP.manuelTurretControl = true;
            hardware.turret.updatePID = true;
        }
    }
    public void run(){
        hardware.turret.updatePID = true;
        hardware.mag.feedTopRing();
        hardware.mag.currentState = Mag.State.TOP;
        hardware.turret.updatePID = true;
        hardware.turret.turretPID.setState(MathFunctions.keepAngleWithin180Degrees(Math.toRadians(0)));
        hardware.turret.updatePID = true;
        sleeep(1500);
        shootPowershot(hardware);
        telemetry.addLine("1st powershot");
        telemetry.update();
        hardware.turret.turretPID.setState(MathFunctions.keepAngleWithin180Degrees(Math.toRadians(-4)));
        hardware.turret.updatePID = true;
        sleeep(1500);
        shootPowershot(hardware);
        telemetry.addLine("2nd powershot");
        telemetry.update();
        hardware.turret.turretPID.setState(MathFunctions.keepAngleWithin180Degrees(Math.toRadians(-9)));
        hardware.turret.updatePID = true;
        sleeep(1500);
        shootPowershot(hardware);
        telemetry.addLine("3rd powershot");
        telemetry.update();
        hardware.turret.updatePID = false;
    }
}

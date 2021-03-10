package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

public abstract class AutoMethods extends LinearOpMode {



    public void shootPowershot(HardwareMecanum hardware) {
        hardware.mag.dropRings();
        sleep(500);//tune timeout
        for(int i = 0; i < 3; i++){
            hardware.mag.pushInRings();
            sleep(300);// tune time
            hardware.mag.setRingPusherResting();
            sleep(250);// tune time
        }
        hardware.mag.collectRings();
    }
    public void shootIndividualRing(HardwareMecanum hardware){
        hardware.mag.pushInRings();
        sleep(550);
        hardware.mag.setRingPusherResting();
    }


}

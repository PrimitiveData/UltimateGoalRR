package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

public abstract class AutoMethods extends LinearOpMode {



    public void shootPowershot(HardwareMecanum hardware) {
        hardware.mag.pushInRings();
        sleep(200);
        hardware.mag.setRingPusherResting();
        sleep(350);
        hardware.mag.updateStateAndSetPosition();
    }


}

package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

@TeleOp(name = "WobbleArm01",group = "TeleOp")
public class wobblearm01 extends LinearOpMode {
    public void runOpMode(){
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
        waitForStart();
        while(!isStopRequested()){
            hardware.wobbler.wobblerArm1.setPosition(0);
            hardware.wobbler.wobblerArm2.setPosition(1);
            hardware.loop();
        }
    }
}

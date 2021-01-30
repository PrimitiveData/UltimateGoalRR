package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

@TeleOp(name = "WobbleArmTester",group = "TeleOp")
public class WobbleArmTester extends LinearOpMode {
    public void runOpMode(){
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
        waitForStart();
        while(!isStopRequested()){
            if(gamepad1.a)
                hardware.wobbler.goToArmRestingPos();
            if(gamepad1.b)
                hardware.wobbler.goToClawRestingPos();
            if(gamepad1.x)
                hardware.wobbler.goToWobblerDropPosition();
            if(gamepad1.y)
                hardware.wobbler.goToWobbleStartingPos();
            if(gamepad1.dpad_down)
                hardware.wobbler.gripWobble();
            if(gamepad1.dpad_up)
                hardware.wobbler.releaseWobble();
        }
    }
}

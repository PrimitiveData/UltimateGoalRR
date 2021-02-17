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
                hardware.wobbler.gripWobble();
            if(gamepad1.b)
                hardware.wobbler.releaseWobble();
            hardware.wobbler.wobblerArm1.setPosition(-gamepad1.left_stick_y);
            hardware.wobbler.wobblerArm2.setPosition(hardware.wobbler.wobblerArm2PositionWhenWobblerArm1IsZero + gamepad1.left_stick_y);
            hardware.telemetry.addData("Wobble Arm Pos: ", hardware.wobbler.wobblerArm1.position);
            hardware.loop();
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

@TeleOp(name = "WobbleArmTester",group = "TeleOp")
public class WobbleArmTester extends LinearOpMode {
    public void runOpMode(){
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
        waitForStart();
        double currentWobblePosition = 0.5;
        while(!isStopRequested()){
            if(gamepad1.a)
                hardware.wobbler.gripWobble();
            if(gamepad1.b)
                hardware.wobbler.releaseWobble();
            currentWobblePosition += gamepad1.left_stick_y*0.001;
            hardware.wobbler.wobblerArm1.setPosition(currentWobblePosition);
            hardware.wobbler.wobblerArm2.setPosition(hardware.wobbler.wobblerArm2PositionWhenWobblerArm1IsZero -currentWobblePosition);
            hardware.telemetry.addData("Wobble Arm Pos: ", currentWobblePosition);
            hardware.loop();
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

@TeleOp(name = "TurretTester",group = "TeleOp")
public class TurretTester extends LinearOpMode {
    public void runOpMode(){
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
        waitForStart();
        while(!isStopRequested()){
            hardware.turret.turretMotor.setPower(gamepad1.right_stick_y);
            hardware.loop();
        }
    }
}

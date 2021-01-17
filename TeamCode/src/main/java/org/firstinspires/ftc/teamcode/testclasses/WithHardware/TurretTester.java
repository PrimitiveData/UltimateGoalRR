package org.firstinspires.ftc.teamcode.testclasses.WithHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp(name = "TurretTester", group = "TeleOp")
public class TurretTester extends LinearOpMode {
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap,telemetry);
        waitForStart();
        while(!isStopRequested()) {
            double output = -gamepad1.left_stick_y;
            telemetry.addData("output",output);
            telemetry.addData("turretPosition",-hardware.turret.encoder.getCurrentPosition());
            telemetry.update();
            hardware.turret.setAllTurretServoPowers(gamepad1.left_stick_y);
            hardware.loop();
        }
    }
}

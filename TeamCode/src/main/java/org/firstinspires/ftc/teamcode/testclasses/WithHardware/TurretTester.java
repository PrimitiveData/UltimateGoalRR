package org.firstinspires.ftc.teamcode.testclasses.WithHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

public class TurretTester extends LinearOpMode {
    public void runOpMode(){
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap,telemetry);
        waitForStart();
        while(!isStopRequested()) {
            double output = -gamepad1.left_stick_y;
            telemetry.addData("output",output);
            telemetry.addData("turretPosition",-hardware.turret.turretMotor.getCurrentPosition());
            telemetry.update();
            hardware.turret.setTurretMotorPower(gamepad1.left_stick_y);
            hardware.loop();
        }
    }
}

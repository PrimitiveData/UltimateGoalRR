package org.firstinspires.ftc.teamcode.testclasses.WithMecanumHardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.PID.PIDwithBasePower;

@Disabled
public class PrintTurretPosition extends LinearOpMode {
    public void runOpMode(){
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
        hardware.turret.turretMotor.readRequested=true;
        waitForStart();
        while(!isStopRequested()) {
            telemetry.addData("heading: ", Math.toDegrees(hardware.turret.localTurretAngleRadians()));
            telemetry.update();
            hardware.loop();
        }
    }
}

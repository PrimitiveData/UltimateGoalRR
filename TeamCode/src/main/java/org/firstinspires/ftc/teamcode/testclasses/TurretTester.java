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
            boolean turretTracking = false;
            boolean magTracking = false;
            double globalTurretAngle = 0;
            boolean aToggle = false;
            boolean bToggle = false;

            if(gamepad1.dpad_left)
                globalTurretAngle += 20;
            if(gamepad1.dpad_right)
                globalTurretAngle -= 20;

            if(gamepad1.a) {
                if(!aToggle)
                    turretTracking = !turretTracking;
                aToggle = true;
            }
            else{
                if(aToggle)
                    aToggle = false;
            }

            if(gamepad1.b) {
                if(!bToggle)
                    magTracking = !magTracking;
                bToggle = true;
            }
            else{
                if(bToggle)
                    bToggle = false;
            }

            if(turretTracking){
                hardware.turret.updatePID = true;
                hardware.turret.setTurretAngle(globalTurretAngle);
            }
            else {
                hardware.turret.updatePID = false;
                hardware.turret.setTurretMotorPower(0);
            }
            if(magTracking) {
                hardware.mag.dropRings();
                hardware.turret.setTurretAngle(globalTurretAngle);
            }
            else
                hardware.mag.collectRings();

            telemetry.addData("Global Turret Angle: ", globalTurretAngle);
            telemetry.addData("Turret Current Local Position: ", hardware.turret.localTurretAngleRadians());
            telemetry.addData("Robot Heading: ", hardware.getAngle());
            telemetry.addData("Turret Tracking: ", turretTracking);
            telemetry.addData("Mag Tracking: ", magTracking);
            telemetry.update();
            hardware.loop();
        }
    }
}

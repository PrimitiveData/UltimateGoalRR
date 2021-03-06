package org.firstinspires.ftc.teamcode.testclasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop.Multithreads.MagFlickerController;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;


@TeleOp(name = "ShooterRegression",group = "TeleOp")
public class ShooterRegression extends LinearOpMode {
    public void runOpMode(){
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(hardware, this);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        hardware.turret.turretMotor.readRequested = true;
        waitForStart();

        boolean turretTracking = false;
        boolean magTracking = false;
        double localTurretAngle = 0;
        boolean aToggle = false;
        boolean bToggle = false;
        double rampPos = 0.2;
        boolean shooterOnTogglePrevLoop = false;
        boolean shooterOn = false;
        double shooterVelo = 1350;

        double increment = 0.012;
        double threshold = 990;

        hardwareThreadInterface.start();
        while(!isStopRequested()){

            if(gamepad1.dpad_left)
                localTurretAngle += Math.toRadians(0.1);
            if(gamepad1.dpad_right)
                localTurretAngle -= Math.toRadians(0.1);

            if(gamepad1.dpad_up)
                rampPos += 0.001;
            if(gamepad1.dpad_down)
                rampPos -= 0.001;
            hardware.shooter.setRampPosition(rampPos);

            if(gamepad1.x){
                hardware.mag.dropRings();
            }

            if(gamepad1.right_bumper){
                for(int i = 0; i < 3; i++){
                    if(-hardware.shooter.shooterMotor1.getVelocity() >= threshold){
                        hardware.mag.pushInRingsThreadBypass();
                        sleep(100);
                        hardware.mag.setRingPusherRestingThreadBypass();
                        sleep(100);
                        hardware.shooter.setRampPosition(hardware.shooter.rampPostion + increment);
                    }
                    else
                        i--;
                }
            }

            if(gamepad2.dpad_up){
                threshold += 2;
            }

            if(gamepad2.dpad_down){
                threshold -= 2;
            }

            if(gamepad2.y){
                increment += 0.002;
            }

            if(gamepad2.a){
                increment -= 0.002;
            }

            if(gamepad1.right_trigger > 0){
                hardware.mag.setRingPusherResting();
            }

            if(gamepad1.left_bumper) {
                if(!shooterOnTogglePrevLoop) {
                    shooterOn = !shooterOn;
                    if(shooterOn){
                        hardware.shooter.firstUpdateShooterPIDFLoop = true;
                    }
                }
                shooterOnTogglePrevLoop = true;
            }
            else{
                if(shooterOnTogglePrevLoop){
                    shooterOnTogglePrevLoop = false;
                }
            }
            shooterVelo -= gamepad1.left_stick_y * 0.1;
            if(shooterOn){
                hardware.shooter.updatePID = true;
                hardware.shooter.shooterVeloPID.setState(shooterVelo);
            /*double voltage = VelocityPIDDrivetrain.getBatteryVoltage();
            double maxVolts = -10.5;
            hardware.shooter.shooterMotor2.setPower(maxVolts/voltage);
            hardware.shooter.shooterMotor1.setPower(maxVolts/voltage);*/
            }
            else{
                hardware.shooter.updatePID = false;
                hardware.shooter.shooterMotor2.setPower(0);
                hardware.shooter.shooterMotor1.setPower(0);
                hardware.shooter.shooterVeloPID.setState(0);
            }

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
                hardware.turret.setLocalTurretAngle(localTurretAngle);
            }
            else {
                hardware.turret.updatePID = false;
                hardware.turret.setTurretMotorPower(0);
            }
            if(magTracking) {
                hardware.mag.dropRings();
            }
            else
                hardware.mag.collectRings();

            telemetry.addData("Global Turret Angle: ", Math.toDegrees(localTurretAngle));
            telemetry.addData("Shooter Velo:  ", shooterVelo);
            telemetry.addData("Flap Position: ", rampPos);
            telemetry.addData("Threshold: ", threshold);
            telemetry.addData("Increment: ", increment);
            telemetry.addData("Turret Current Local Position: ", Math.toDegrees(hardware.turret.localTurretAngleRadians()));
            packet.put("Turret Angle: ", Math.toDegrees(hardware.turret.localTurretAngleRadians()));
            packet.put("Shooter Velo: ", shooterVelo);
            packet.put("Flap Position: ", rampPos);
            packet.put("Threshold: ", threshold);
            packet.put("Increment: ", increment);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            sleep(10);
        }
    }
}

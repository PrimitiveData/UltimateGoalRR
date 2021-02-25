package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop.Multithreads.MagFlickerController;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

@TeleOp(name = "ShooterRegression",group = "TeleOp")
public class ShooterRegression extends LinearOpMode {
    public void runOpMode(){
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
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
        double shooterVelo = 1450;
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
                hardware.mag.pushInRings();
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
            telemetry.addData("Hardware Angle: ", hardware.getAngle());
            telemetry.addData("Turret Current Local Position: ", Math.toDegrees(hardware.turret.localTurretAngleRadians()));
            telemetry.addData("Robot Heading: ", hardware.getAngle());
            telemetry.addData("Turret Tracking: ", turretTracking);
            telemetry.addData("Mag Tracking: ", magTracking);
            telemetry.addData("Shooter On: ", shooterOn);
            telemetry.update();
            hardware.loop();
        }
    }
}

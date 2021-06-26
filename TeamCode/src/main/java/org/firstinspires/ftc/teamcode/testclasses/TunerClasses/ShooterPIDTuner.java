package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

@TeleOp(group = "TeleOp", name = "ShooterPIDTuner")
public class ShooterPIDTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
        hardware.shooter.updatePID = true;
        waitForStart();
        while(!isStopRequested()) {
            if(gamepad1.y){
                hardware.shooter.shooterVeloPID.setState(1200);
            }
            else if(gamepad1.x){
                hardware.shooter.shooterVeloPID.setState(0);
            }
            double kPChange = -gamepad1.left_stick_y * 0.005;
            double kIChange = -gamepad1.right_stick_y * 0.005;
            double kDChange = -gamepad2.left_stick_y * 0.005;
            telemetry.addLine("kP: "+hardware.shooter.shooterVeloPID.kP+", kDChange: "+hardware.shooter.shooterVeloPID.kD+", kIChange: "+hardware.shooter.shooterVeloPID.kI);
            hardware.shooter.shooterVeloPID.kP = hardware.shooter.shooterVeloPID.kP + kPChange;
            hardware.shooter.shooterVeloPID.kI = hardware.shooter.shooterVeloPID.kI + kIChange;
            hardware.shooter.shooterVeloPID.kD = hardware.shooter.shooterVeloPID.kD + kDChange;
            packet.put("targetHeading: ", hardware.shooter.shooterVeloPID.desiredState);
            packet.put("currentHeading: ", hardware.shooter.shooterVeloPID.currentState);
            //dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            hardware.loop();
        }
}
}

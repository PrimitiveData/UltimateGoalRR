package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.PID.PIDwithBasePower;
@Config
@TeleOp(group = "TeleOp", name = "TurretPIDTuner")
public class TurretPIDTuner extends LinearOpMode {
    public static double kP = 5;
    public static double kI = 5;
    public static double kD = 0.006;

    private boolean move = false;
    ElapsedTime timer = new ElapsedTime();

    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
        hardware.turret.turretMotor.readRequested=true;
        waitForStart();
        hardware.turret.setLocalTurretAngle(Math.toRadians(90));
        hardware.turret.updatePID = true;
        timer.reset();
        while(!isStopRequested()) {
            if(gamepad1.y) {
                move = true;
                timer.reset();
            }
            if(gamepad1.x)
                move = false;
            if(move){
                if(timer.seconds() < 0.8)
                    hardware.turret.setLocalTurretAngle(0);
                else if(timer.seconds() < 1.6 && timer.seconds() > 0.8)
                    hardware.turret.setLocalTurretAngle(Math.PI/2);
                else
                    timer.reset();
            }
            /*
            double kPChange = gamepad1.left_stick_y * 0.005;
            double kIChange = gamepad1.right_stick_y * 0.005;
            double kDChange = gamepad2.left_stick_y * 0.005;
            double kStaticChange = gamepad2.right_stick_y * 0.005;
            */
            //telemetry.addLine("kPChange: "+kPChange+", kDChange: "+kDChange +", kIChange: "+kIChange+", kStaticChange: "+kStaticChange);
            hardware.turret.turretPID.kP = kP; //hardware.turret.turretPID.kP + kPChange;
            hardware.turret.turretPID.kI = kI; //hardware.turret.turretPID.kI + kIChange;
            hardware.turret.turretPID.kD = kD; //hardware.turret.turretPID.kD + kDChange;
            //hardware.turret.turretPID.kStatic = hardware.turret.turretPID.kStatic + kStaticChange;
            telemetry.addLine("kP: "+hardware.turret.turretPID.kP+", kD: "+hardware.turret.turretPID.kD +", kI: "+hardware.turret.turretPID.kI+", kStatic: "+hardware.turret.turretPID.kStatic);
            telemetry.addData("heading: ", Math.toDegrees(hardware.turret.localTurretAngleRadians()));
            double output = hardware.turret.turretPID.updateCurrentStateAndGetOutput(hardware.turret.localTurretAngleRadians());
            telemetry.addData("output: ",output);
            telemetry.addData("currentIntegral: ",hardware.turret.turretPID.integral);
            telemetry.addData("targetHeading: ", hardware.turret.turretPID.desiredState);
            telemetry.addData("currentHeading: ", hardware.turret.turretPID.currentState);
            packet.put("targetHeading: ", hardware.turret.turretPID.desiredState);
            packet.put("currentHeading: ", hardware.turret.turretPID.currentState);
            System.out.println(hardware.turret.turretPID.desiredState - hardware.turret.turretPID.currentState);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            hardware.loop();
        }
    }
}

package org.firstinspires.ftc.teamcode.Auto.Multithreads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

public class AutoAim extends Thread {
    HardwareMecanum hardware;

    Telemetry telemetry;
    LinearOpMode parentOP;
    public boolean stopRequested;
    public AutoAim(HardwareMecanum hardware, Telemetry telemetry, LinearOpMode parentOP){
        this.telemetry = telemetry;
        this.hardware = hardware;
        this.parentOP = parentOP;
        stopRequested = false;
    }
    public void run(){
        while(!parentOP.isStopRequested()&&!stopRequested) {
            double[] turretPosition = MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(), hardware.getYAbsoluteCenter(), -4.72974566929, hardware.getAngle());
            telemetry.addLine("Turret Position: " + turretPosition[0] + ", " + turretPosition[1]);
            double distanceToGoal = Math.hypot(turretPosition[1] - FieldConstants.highGoalPosition[1], turretPosition[0] - FieldConstants.highGoalPosition[0]);
            double angleToGoal = Math.atan2(FieldConstants.highGoalPosition[1] - turretPosition[1], FieldConstants.highGoalPosition[0] - turretPosition[0]) + hardware.turret.getTurretOffset(distanceToGoal);
            angleToGoal+=-0.1;
            telemetry.addData("angleToGoal", Math.toDegrees(angleToGoal));
            hardware.shooter.autoRampPositionForHighGoal(distanceToGoal);
            hardware.turret.setTurretAngle(angleToGoal);
            telemetry.update();
        }
    }
}

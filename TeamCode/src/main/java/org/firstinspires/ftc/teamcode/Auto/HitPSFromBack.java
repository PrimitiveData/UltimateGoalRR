package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.AutoAim;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.AutoAimVelo;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.CloseTheCamera;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.WiggleTheMag;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.Ramsete.Pose;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;
import org.firstinspires.ftc.teamcode.hardware.PID.ShooterPID;
import org.firstinspires.ftc.teamcode.vision.UltimateGoalReturnPositionPipeline;

import java.util.Arrays;
import java.util.Vector;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

@Disabled
public class HitPSFromBack extends AutoMethods {
    int stack = 2;
    OpenCvCamera webcam;
    String powershotLogTag = "powershotLog";
    double powerShotVelo = 1275;
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        UltimateGoalReturnPositionPipeline pipeline = new UltimateGoalReturnPositionPipeline();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        webcam.resumeViewport();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        sleep(2000);
        stack = pipeline.stack;
        telemetry.addData("stack", stack);
        telemetry.update();


        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry, false);
        hardware.shooter.shooterVeloPID = new ShooterPID(0.1, 0.5, 0, 0.004893309156, 3.238478883, 0, Double.POSITIVE_INFINITY, hardware.time, "/sdcard/FIRST/shooterFFdata.txt");
        HardwareThreadInterface hardwareThreadInterface = new HardwareThreadInterface(hardware, this);
        hardware.turret.turretMotor.readRequested = true;

        hardware.wobbler.goToWobbleStartingPos();
        hardware.wobbler.gripWobble();
        hardware.mag.setRingPusherResting();
        hardware.mag.dropRings();
        hardware.intake.holdIntakeUp();
        hardware.loop();
        waitForStart();
        hardware.shooter.setRampPosition(0);
        hardwareThreadInterface.start();
        CloseTheCamera closeCamera = new CloseTheCamera(webcam);
        closeCamera.start();
        hardware.shooter.updatePID = true;
        hardware.shooter.shooterVeloPID.setState(powerShotVelo);
        hardware.shooter.setRampPosition(0.27);
        hardware.turret.updatePID = true;
        hardware.wobbler.raiseWobble();
        double ps2TurretAngle = Math.toRadians(-180);
        double ps3TurretAngle = Math.toRadians(-182);
        double ps1TurretAngle = Math.toRadians(-186);
        hardware.shooter.shooterVeloPID.powershotAntiWindup = true;
        hardware.turret.setLocalTurretAngleAuto(ps2TurretAngle);
        sleep(3000);
        ElapsedTime powershotTimer = new ElapsedTime();
        ElapsedTime powershotForcedExitTimer = new ElapsedTime();
        double prevTurretAngle = hardware.turret.localTurretAngleRadians();
        powershotForcedExitTimer.reset();
        while (!isStopRequested()) {
            double currentTurretAngle = hardware.turret.localTurretAngleRadians();
            if (Math.abs(currentTurretAngle - prevTurretAngle) > Math.toRadians(0.08))
                powershotTimer.reset();
            if ((powershotTimer.milliseconds() >= 80 && Math.abs(currentTurretAngle - ps2TurretAngle) < Math.toRadians(0.25)) || powershotForcedExitTimer.milliseconds() > 1500)
                break;
            prevTurretAngle = currentTurretAngle;
        }
        RobotLog.dd(powershotLogTag, "Powershot 1, Desired Angle: " + Math.toDegrees(ps2TurretAngle) + ", Current Angle: " + Math.toDegrees(hardware.turret.localTurretAngleRadians()));
        shootIndividualRing(hardware);
        RobotLog.dd(powershotLogTag, "Powershot 1.2, Desired Angle: " + Math.toDegrees(ps2TurretAngle) + ", Current Angle: " + Math.toDegrees(hardware.turret.localTurretAngleRadians()));
        hardware.turret.setLocalTurretAngleAuto(ps3TurretAngle);
        prevTurretAngle = hardware.turret.localTurretAngleRadians();
        powershotForcedExitTimer.reset();
        while (!isStopRequested()) {
            double currentTurretAngle = hardware.turret.localTurretAngleRadians();
            if (Math.abs(currentTurretAngle - prevTurretAngle) > Math.toRadians(0.08))
                powershotTimer.reset();
            if ((powershotTimer.milliseconds() >= 80 && Math.abs(currentTurretAngle - ps3TurretAngle) < Math.toRadians(0.25)) || powershotForcedExitTimer.milliseconds() > 2000)
                break;
            prevTurretAngle = currentTurretAngle;
        }
        RobotLog.dd(powershotLogTag, "Powershot 2, Desired Angle: " + Math.toDegrees(ps3TurretAngle) + ", Current Angle: " + Math.toDegrees(hardware.turret.localTurretAngleRadians()));
        shootIndividualRing(hardware);
        RobotLog.dd(powershotLogTag, "Powershot 2.2, Desired Angle: " + Math.toDegrees(ps3TurretAngle) + ", Current Angle: " + Math.toDegrees(hardware.turret.localTurretAngleRadians()));
        hardware.turret.setLocalTurretAngleAuto(ps1TurretAngle);
        prevTurretAngle = hardware.turret.localTurretAngleRadians();
        powershotForcedExitTimer.reset();
        while (!isStopRequested()) {
            double currentTurretAngle = hardware.turret.localTurretAngleRadians();
            if (Math.abs(currentTurretAngle - prevTurretAngle) > Math.toRadians(0.08))
                powershotTimer.reset();
            if ((powershotTimer.milliseconds() >= 80 && Math.abs(currentTurretAngle - ps1TurretAngle) < Math.toRadians(0.25)) || powershotForcedExitTimer.milliseconds() > 2000)
                break;
            prevTurretAngle = currentTurretAngle;
        }
        RobotLog.dd(powershotLogTag, "Powershot 3, Desired Angle: " + Math.toDegrees(ps1TurretAngle) + ", Current Angle: " + Math.toDegrees(hardware.turret.localTurretAngleRadians()));
        shootIndividualRing(hardware);
        RobotLog.dd(powershotLogTag, "Powershot 3.2, Desired Angle: " + Math.toDegrees(ps1TurretAngle) + ", Current Angle: " + Math.toDegrees(hardware.turret.localTurretAngleRadians()));
    }
}
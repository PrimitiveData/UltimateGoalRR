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
public class UltimateGoalRedAuto extends AutoMethods {
    int stack = 2;
    OpenCvCamera webcam;
    String powershotLogTag = "powershotLog";
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        UltimateGoalReturnPositionPipeline pipeline = new UltimateGoalReturnPositionPipeline();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        webcam.resumeViewport();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        sleep(2000);
        stack = pipeline.stack;
        telemetry.addData("stack",stack);
        telemetry.update();


        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry, false);
        hardware.shooter.shooterVeloPID = new ShooterPID(0,0,0,0.004893309156,3.238478883,0,Double.POSITIVE_INFINITY,hardware.time,"/sdcard/FIRST/shooterFFdata.txt");
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(hardware, this);
        hardware.turret.turretMotor.readRequested = true;

        hardware.drive.accelConstraint = new ProfileAccelerationConstraint(30);
        Trajectory goToShootPos = hardware.drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-60,0))
                .build();
        hardware.drive.accelConstraint = new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL);
        Trajectory pickUpRingsPrelude = hardware.drive.trajectoryBuilder(goToShootPos.end())
                .lineToConstantHeading(new Vector2d(goToShootPos.end().getX(), goToShootPos.end().getY()+20))
                .build();

        hardware.drive.velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(7, TRACK_WIDTH)
        ));
        double distanceToPickUp= 14;
        double headingToPickUp = Math.toRadians(0);
        Trajectory pickUpRings = hardware.drive.trajectoryBuilder(pickUpRingsPrelude.end())
                .lineToConstantHeading(new Vector2d(pickUpRingsPrelude.end().getX() + distanceToPickUp*Math.cos(headingToPickUp),pickUpRingsPrelude.end().getY() + distanceToPickUp*Math.sin(headingToPickUp)))
                .build();

        double distanceToPickUp2 = 5;
        Trajectory pickUpRings2 = hardware.drive.trajectoryBuilder(pickUpRings.end())
                .lineToConstantHeading(new Vector2d(pickUpRings.end().getX()+distanceToPickUp2*Math.cos(headingToPickUp),pickUpRings.end().getY() + distanceToPickUp2*Math.sin(headingToPickUp)))
                .build();

        hardware.drive.velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(2, TRACK_WIDTH)
        ));

        hardware.drive.velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)));

        Trajectory dropWobbler1 = null;
        if(stack==0) {
            dropWobbler1 = hardware.drive.trajectoryBuilder(goToShootPos.end())
                    .splineToLinearHeading(new Pose2d(-86,24,-Math.toRadians(90)),Math.toRadians(90))
                    .build();

        }
        else if(stack == 1){
            dropWobbler1 = hardware.drive.trajectoryBuilder(pickUpRings.end())
                    .lineToConstantHeading(new Vector2d(-81,14))
                    .build();
        }
        else{
            dropWobbler1 = hardware.drive.trajectoryBuilder(pickUpRings2.end())
                    .lineToConstantHeading(new Vector2d(-111,36))
                    .build();
        }
        Trajectory collect2ndWobbler = null;
        if(stack==0) {
            collect2ndWobbler = hardware.drive.trajectoryBuilder(new Pose2d(dropWobbler1.end().getX(), dropWobbler1.end().getY(), MathFunctions.keepAngleWithin180Degrees(dropWobbler1.end().getHeading())))
                    .splineToSplineHeading(new Pose2d(-65,24,Math.toRadians(-179)),0)
                    .splineToSplineHeading(new Pose2d(-30,39,Math.toRadians(-179)),0)
                    .build();
        }
        else if(stack == 1){
            collect2ndWobbler = hardware.drive.trajectoryBuilder(new Pose2d(dropWobbler1.end().getX(),dropWobbler1.end().getY(),-Math.toRadians(179)))
                    .lineToConstantHeading(new Vector2d(-30,39))
                    .build();
        }else{
            collect2ndWobbler = hardware.drive.trajectoryBuilder(new Pose2d(dropWobbler1.end().getX(),dropWobbler1.end().getY(),-Math.toRadians(179)))
                    .lineToConstantHeading(new Vector2d(-28,39))
                    .build();
        }

        Trajectory dropWobbler2 = null;
        if(stack==0) {
            dropWobbler2 = hardware.drive.trajectoryBuilder(collect2ndWobbler.end())
                    .splineToLinearHeading(new Pose2d(-82,24,-Math.toRadians(90)),Math.toRadians(135))
                    .build();
        }else if(stack==1){
            dropWobbler2 = hardware.drive.trajectoryBuilder(collect2ndWobbler.end())
                    .splineToLinearHeading(new Pose2d(-77,12,0),Math.toRadians(135))
                    .build();
        }else{
            dropWobbler2 = hardware.drive.trajectoryBuilder(collect2ndWobbler.end())
                    .lineToSplineHeading(new Pose2d(-65,15,0))
                    .splineToSplineHeading(new Pose2d(-104,33,0),Math.toRadians(135))
                    .build();
        }

        Trajectory park = null;
        if(stack==0) {
            park = hardware.drive.trajectoryBuilder(dropWobbler2.end())
                    .lineToConstantHeading(new Vector2d(-69,20))
                    .build();
        }
        else if(stack == 1){
            park = hardware.drive.trajectoryBuilder(dropWobbler2.end())
                    .lineToConstantHeading(new Vector2d(-69,12))
                    .build();
        }
        else if(stack == 2){
            park = hardware.drive.trajectoryBuilder(dropWobbler2.end())
                    .lineToConstantHeading(new Vector2d(-82,28))
                    .build();
        }

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
        hardware.shooter.shooterVeloPID.setState(1200);
        hardware.shooter.setRampPosition(0.27);
        hardware.turret.updatePID = true;
        hardware.wobbler.raiseWobble();
        double ps1TurretAngle=-Math.toRadians(188.5);
        double ps3TurretAngle=-Math.toRadians(182);
        double ps2TurretAngle=-Math.toRadians(176);
        hardware.turret.setLocalTurretAngleAuto(ps2TurretAngle);
        double goToShootPosStartTime = hardware.time.milliseconds();
        hardware.drive.followTrajectoryAsync(goToShootPos);
        while(hardware.drive.isBusy() && !isStopRequested()){
            if(hardware.time.milliseconds() - goToShootPosStartTime > 750 && hardware.time.milliseconds() - goToShootPosStartTime < 850){
                hardware.intake.dropIntake();
            }
            if(hardware.time.milliseconds() - goToShootPosStartTime > 1550 && hardware.time.milliseconds() - goToShootPosStartTime < 1650){
                hardware.intake.raiseBumper();
            }
            sleep(10);
        }
        hardware.shooter.shooterVeloPID.powershotAntiWindup = true;
        hardware.intake.dropIntake();
        ps2TurretAngle = ps2TurretAngle - hardware.drive.getRawExternalHeading();
        hardware.turret.setLocalTurretAngleAuto(ps2TurretAngle);
        ElapsedTime powershotTimer = new ElapsedTime();
        ElapsedTime powershotForcedExitTimer = new ElapsedTime();
        double prevTurretAngle = hardware.turret.localTurretAngleRadians();
        powershotForcedExitTimer.reset();
        while(!isStopRequested()) {
            double currentTurretAngle = hardware.turret.localTurretAngleRadians();
            if (Math.abs(currentTurretAngle - prevTurretAngle) > Math.toRadians(0.08))
                powershotTimer.reset();
            if ((powershotTimer.milliseconds() >= 80 && Math.abs(currentTurretAngle - ps2TurretAngle) < Math.toRadians(0.25))||powershotForcedExitTimer.milliseconds() > 1500)
                break;
            prevTurretAngle = currentTurretAngle;
        }
        RobotLog.dd(powershotLogTag,"Powershot 1, Desired Angle: " + Math.toDegrees(ps2TurretAngle) + ", Current Angle: " + Math.toRadians(hardware.turret.localTurretAngleRadians()));
        shootIndividualRing(hardware);
        RobotLog.dd(powershotLogTag,"Powershot 1.2, Desired Angle: " + Math.toDegrees(ps2TurretAngle) + ", Current Angle: " + Math.toRadians(hardware.turret.localTurretAngleRadians()));
        ps3TurretAngle = ps3TurretAngle - hardware.drive.getRawExternalHeading();
        hardware.turret.setLocalTurretAngleAuto(ps3TurretAngle);
        prevTurretAngle = hardware.turret.localTurretAngleRadians();
        powershotForcedExitTimer.reset();
        while(!isStopRequested()) {
            double currentTurretAngle = hardware.turret.localTurretAngleRadians();
            if (Math.abs(currentTurretAngle - prevTurretAngle) > Math.toRadians(0.08))
                powershotTimer.reset();
            if ((powershotTimer.milliseconds() >= 80 && Math.abs(currentTurretAngle - ps3TurretAngle) < Math.toRadians(0.25))||powershotForcedExitTimer.milliseconds() > 2000)
                break;
            prevTurretAngle = currentTurretAngle;
        }
        RobotLog.dd(powershotLogTag,"Powershot 2, Desired Angle: " + Math.toDegrees(ps3TurretAngle) + ", Current Angle: " + Math.toDegrees(hardware.turret.localTurretAngleRadians()));
        shootIndividualRing(hardware);
        RobotLog.dd(powershotLogTag,"Powershot 2.2, Desired Angle: " + Math.toDegrees(ps3TurretAngle) + ", Current Angle: " + Math.toDegrees(hardware.turret.localTurretAngleRadians()));
        ps1TurretAngle = ps1TurretAngle - hardware.drive.getRawExternalHeading();
        hardware.turret.setLocalTurretAngleAuto(ps1TurretAngle);
        prevTurretAngle = hardware.turret.localTurretAngleRadians();
        powershotForcedExitTimer.reset();
        while(!isStopRequested()) {
            double currentTurretAngle = hardware.turret.localTurretAngleRadians();
            if (Math.abs(currentTurretAngle - prevTurretAngle) > Math.toRadians(0.08))
                powershotTimer.reset();
            if ((powershotTimer.milliseconds() >= 80 && Math.abs(currentTurretAngle - ps1TurretAngle) < Math.toRadians(0.25))||powershotForcedExitTimer.milliseconds() > 2000)
                break;
            prevTurretAngle = currentTurretAngle;
        }
        RobotLog.dd(powershotLogTag,"Powershot 3, Desired Angle: " + Math.toDegrees(ps1TurretAngle) + ", Current Angle: " + Math.toDegrees(hardware.turret.localTurretAngleRadians()));
        shootIndividualRing(hardware);
        RobotLog.dd(powershotLogTag,"Powershot 3.2, Desired Angle: " + Math.toDegrees(ps1TurretAngle) + ", Current Angle: " + Math.toDegrees(hardware.turret.localTurretAngleRadians()));
        hardware.shooter.shooterVeloPID.clearI();
        hardware.shooter.shooterVeloPID.kP = 0.1;
        hardware.shooter.shooterVeloPID.kI = 0.5;
        AutoAimVelo autoAim = null;
        WiggleTheMag wiggleTheMag = null;
        hardware.shooter.shooterVeloPID.powershotAntiWindup = false;
        double prevMaxPositiveTurret = hardware.turret.maxPositive;
        if(stack == 2 || stack == 1) {
            hardware.mag.collectRings();
            hardware.intake.turnIntake(1);
            hardware.turret.maxPositive = Math.toRadians(0);
            autoAim = new AutoAimVelo(hardware, telemetry, this, 1225);
            autoAim.start();
            wiggleTheMag = new WiggleTheMag(hardware, this);
            wiggleTheMag.start();
            hardware.drive.followTrajectoryAsync(pickUpRingsPrelude);
            while (hardware.drive.isBusy() && !isStopRequested()) {
                sleep(1);
            }
            hardware.drive.followTrajectoryAsync(pickUpRings);
            while (hardware.drive.isBusy() && !isStopRequested()) {
                sleep(1);
            }
        /*for(int i = 0; i < 4; i++){
            hardware.drive.setWeightedDrivePower(new Pose2d(-1, 0, 0));
            sleep(250);
            hardware.drive.setWeightedDrivePower(new Pose2d(1, 0, 0));
            sleep(250);
        }
        hardware.drive.setWeightedDrivePower(new Pose2d(0,0,0));*/
        }
        if(stack == 1){
            sleep(1600);
            hardware.intake.turnIntake(0);
            hardware.turret.setMagAngle(0.5);
            sleep(150);
            if(hardware.mag.currentState == Mag.State.COLLECT) {
                hardware.mag.dropRings();
                sleep(1000);//tune timeout//
            }
            hardware.shooter.rampAngleAdjustmentConstant -= 0.035;
            for(int i = 0; i < 1; i++){
                hardware.mag.pushInRings();
                sleep(400);// tune time
                hardware.mag.setRingPusherResting();
                sleep(400);// tune time
            }
            hardware.mag.collectRings();
            wiggleTheMag.stopRequested = true;
            autoAim.stopRequested = true;
            hardware.turret.maxPositive = prevMaxPositiveTurret;
        }
        if(stack == 2){
            hardware.turret.turretAngleOffsetAdjustmentConstant = Math.toRadians(1);
            sleep(2400);
            hardware.intake.turnIntake(0);
            hardware.turret.setMagAngle(0.5);
            sleep(150);
            if(hardware.mag.currentState == Mag.State.COLLECT) {
                hardware.mag.dropRings();
                sleep(500);//tune timeout//
            }
            hardware.shooter.rampAngleAdjustmentConstant -= 0.035;
            for(int i = 0; i < 3; i++){
                hardware.mag.pushInRingsThreadBypass();
                sleep(250);// tune time
                hardware.mag.setRingPusherRestingThreadBypass();
                sleep(150);// tune time
            }
            hardware.turret.turretAngleOffsetAdjustmentConstant = Math.toDegrees(0);
            hardware.mag.collectRings();
            sleep(500);
            hardware.intake.turnIntake(1);
            hardware.drive.followTrajectoryAsync(pickUpRings2);
            while (hardware.drive.isBusy()&&!isStopRequested()){
                sleep(1);
            }
            /*for(int i = 0; i < 3; i++){
                hardware.drive.setWeightedDrivePower(new Pose2d(-0.4, 0, 0));
                sleep(250);
                hardware.drive.setWeightedDrivePower(new Pose2d(0.4, 0, 0));
                sleep(250);
            }
            hardware.drive.setWeightedDrivePower(new Pose2d(0,0,0));*/
            sleep(700);
            wiggleTheMag.stopRequested = true;
            sleep(250);
            hardware.turret.setMagAngle(hardware.mag.magRotationCollectPosition + 180/540);
            hardware.intake.turnIntake(0);
            hardware.turret.setMagAngle(0.5);
            sleep(150);
            if(hardware.mag.currentState == Mag.State.COLLECT) {
                hardware.mag.dropRings();
                sleep(500);//tune timeout
            }
            hardware.shooter.rampAngleAdjustmentConstant = 0.0;
            for(int i = 0; i < 3; i++){
                hardware.mag.pushInRings();
                sleep(250);// tune time
                hardware.mag.setRingPusherResting();
                sleep(150);// tune time
            }
            hardware.shooter.rampAngleAdjustmentConstant = 0;
            autoAim.stopRequested = true;
            hardware.turret.maxPositive = prevMaxPositiveTurret;
        }
        hardware.wobbler.goToWobblerDropPosition();
        hardware.drive.followTrajectoryAsync(dropWobbler1);
        while(hardware.drive.isBusy()&&!isStopRequested()){
            sleep(1);
            hardware.turret.setLocalTurretAngle(0);
            hardware.shooter.updatePID = false;
            hardware.shooter.shooterMotor1.setPower(0);
            hardware.shooter.shooterMotor2.setPower(0);
        }
        hardware.wobbler.releaseWobble();
        sleep(300);
        hardware.wobbler.goToClawRestingPos();
        if(stack == 2 || stack == 1) {
            hardware.wobbler.raiseWobble();
            hardware.drive.turnAsync(-Math.toRadians(179));
            while (hardware.drive.isBusy() && !isStopRequested()) {
                sleep(1);
            }
        }
        if(stack == 1 || stack == 2) {
            hardware.wobbler.moveArmToGrabPos();
        }
        else if (stack == 0){
            hardware.wobbler.raiseWobble();
        }
        hardware.drive.followTrajectoryAsync(collect2ndWobbler);
        double collect2ndWobblerStartTime = hardware.time.milliseconds();
        while(hardware.drive.isBusy()&&!isStopRequested()){
            if(stack == 0){
                if(hardware.time.milliseconds() > collect2ndWobblerStartTime + 400){
                    hardware.wobbler.moveArmToGrabPos();
                }
            }
            if(hardware.time.milliseconds() > collect2ndWobblerStartTime + 900){
                hardware.wobbler.releaseWobble();
            }
        }
        hardware.drive.turnAsync(MathFunctions.keepAngleWithin180Degrees( -Math.toRadians(179)-hardware.getAngle()));
        while(hardware.drive.isBusy() && !isStopRequested()){
            sleep(1);
        }
        hardware.wobbler.gripWobble();
        sleep(300);
        hardware.wobbler.goToWobblerDropPosition2();

        hardware.drive.followTrajectoryAsync(dropWobbler2);
        while(hardware.drive.isBusy() && !isStopRequested()){
            sleep(1);
        }
        if(stack == 2){
            hardware.wobbler.releaseWobble();
            hardware.wobbler.raiseWobble();
        }else if(stack == 0 || stack == 1) {
            hardware.wobbler.releaseWobble();

            sleep(350);
            hardware.wobbler.goToClawRestingPos();
            hardware.wobbler.raiseWobble();
        }
        hardware.drive.followTrajectoryAsync(park);
        while(hardware.drive.isBusy() && !isStopRequested()){
            sleep(1);
        }

        if(stack == 1) {
            sleep(1000);
            hardware.drive.turnAsync(Math.toRadians(180));
            while (hardware.drive.isBusy() && !isStopRequested()) {
                sleep(1);
            }
        }
        if(stack == 0){
            sleep(1000);
            hardware.wobbler.raiseWobble();
            hardware.drive.turnAsync(Math.toRadians(-90));
            while (hardware.drive.isBusy() && !isStopRequested()){
                sleep(1);
            }
        }
    }
}
package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.AutoAim;
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
import org.firstinspires.ftc.teamcode.vision.UltimateGoalReturnPositionPipeline;

import java.util.Arrays;
import java.util.Vector;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

@Autonomous(name = "RedAuto", group = "Autonomous")
public class UltimateGoalRedAuto extends AutoMethods {
    int stack = 2;
    OpenCvCamera webcam;
    public void runOpMode(){
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        UltimateGoalReturnPositionPipeline pipeline = new UltimateGoalReturnPositionPipeline();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        webcam.resumeViewport();

        sleep(2000);
        stack = pipeline.stack;
        telemetry.addData("stack",stack);
        telemetry.update();
         */
        if(gamepad1.x){
            stack = 0;
        }
        else if(gamepad1.y){
            stack = 1;
        }
        else{
            stack = 2;
        }
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(hardware, this);
        hardware.turret.turretMotor.readRequested = true;
        Trajectory goToShootPos = hardware.drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-60,0))
                .build();

        Trajectory pickUpRingsPrelude = hardware.drive.trajectoryBuilder(goToShootPos.end())
                .lineToConstantHeading(new Vector2d(goToShootPos.end().getX(), goToShootPos.end().getY()+20))
                .build();

        hardware.drive.velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(12, TRACK_WIDTH)
        ));
        double distanceToPickUp= 16;
        double headingToPickUp = Math.toRadians(0);
        Trajectory pickUpRings = hardware.drive.trajectoryBuilder(pickUpRingsPrelude.end())
                .lineToConstantHeading(new Vector2d(pickUpRingsPrelude.end().getX() + distanceToPickUp*Math.cos(headingToPickUp),pickUpRingsPrelude.end().getY() + distanceToPickUp*Math.sin(headingToPickUp)))
                .build();

        double distanceToPickUp2 = 18;
        Trajectory pickUpRings2 = hardware.drive.trajectoryBuilder(pickUpRings.end())
                .lineToConstantHeading(new Vector2d(pickUpRings.end().getX()+distanceToPickUp2*Math.cos(headingToPickUp),pickUpRings.end().getY() + distanceToPickUp2*Math.sin(headingToPickUp)))
                .build();

        hardware.drive.velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));

        Trajectory dropWobbler1 = null;
        if(stack==0) {
            dropWobbler1 = hardware.drive.trajectoryBuilder(goToShootPos.end())
                    .splineToLinearHeading(new Pose2d(-88,24,-Math.toRadians(90)),Math.toRadians(90))
                    .build();

        }
        else if(stack == 1){
            dropWobbler1 = hardware.drive.trajectoryBuilder(pickUpRings.end())
                    .lineToConstantHeading(new Vector2d(-84,14))
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
                    .splineToLinearHeading(new Pose2d(-104,36,0),Math.toRadians(135))
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
                    .lineToConstantHeading(new Vector2d(-69,28))
                    .build();
        }

        hardware.wobbler.goToWobbleStartingPos();
        hardware.wobbler.gripWobble();
        hardware.mag.setRingPusherResting();
        hardware.mag.dropRings();
        hardware.loop();
        waitForStart();
        hardware.shooter.setRampPosition(0);
        hardware.intake.dropIntake();
        hardwareThreadInterface.start();
        //CloseTheCamera closeCamera = new CloseTheCamera(webcam);
        //closeCamera.start();
        hardware.shooter.updatePID = true;
        hardware.shooter.shooterVeloPID.setState(1300);
        hardware.turret.updatePID = true;
        hardware.wobbler.raiseWobble();
        double ps1TurretAngle=-Math.toRadians(174);
        double ps3TurretAngle=-Math.toRadians(180);
        double ps2TurretAngle=-Math.toRadians(184.5);
        hardware.turret.setLocalTurretAngleAuto(ps2TurretAngle);
        double goToShootPosStartTime = hardware.time.milliseconds();
        hardware.drive.followTrajectoryAsync(goToShootPos);
        while(hardware.drive.isBusy() && !isStopRequested()){
            if(hardware.time.milliseconds() - goToShootPosStartTime > 450 && hardware.time.milliseconds() - goToShootPosStartTime < 550){
                hardware.intake.raiseBumper();
            }
            sleep(1);
        }
        hardware.intake.dropIntake();
        shootIndividualRing(hardware);
        hardware.turret.setLocalTurretAngleAuto(ps3TurretAngle);
        while(Math.abs(hardware.turret.localTurretAngleRadians() - ps3TurretAngle) > Math.toRadians(0.25) && !isStopRequested())
            sleep(1);
        ElapsedTime powershotTimer = new ElapsedTime();
        double prevTurretAngle = hardware.turret.localTurretAngleRadians();
        while(!isStopRequested()) {
            double currentTurretAngle = hardware.turret.localTurretAngleRadians();
            if (Math.abs(currentTurretAngle - prevTurretAngle) > Math.toRadians(0.125))
                powershotTimer.reset();
            if (powershotTimer.milliseconds() >= 150 && Math.abs(currentTurretAngle - ps3TurretAngle) < Math.toRadians(0.25))
                break;
            prevTurretAngle = currentTurretAngle;
        }
        shootIndividualRing(hardware);
        hardware.turret.setLocalTurretAngleAuto(ps1TurretAngle);
        prevTurretAngle = hardware.turret.localTurretAngleRadians();
        while(!isStopRequested()) {
            double currentTurretAngle = hardware.turret.localTurretAngleRadians();
            if (Math.abs(currentTurretAngle - prevTurretAngle) > Math.toRadians(0.125))
                powershotTimer.reset();
            if (powershotTimer.milliseconds() >= 150 && Math.abs(currentTurretAngle - ps1TurretAngle) < Math.toRadians(0.25))
                break;
            prevTurretAngle = currentTurretAngle;
        }
        shootIndividualRing(hardware);
        AutoAim autoAim = null;
        double prevMaxPositiveTurret = hardware.turret.maxPositive;
        WiggleTheMag wiggleTheMag = null;
        if(stack == 2 || stack == 1) {

            hardware.mag.collectRings();
            hardware.intake.turnIntake(1);
            hardware.turret.maxPositive = Math.toRadians(0);
            autoAim = new AutoAim(hardware, telemetry, this);
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
            sleep(1250);
            hardware.intake.turnIntake(0);
            if(hardware.mag.currentState == Mag.State.COLLECT) {
                hardware.mag.dropRings();
                sleep(500);//tune timeout//
            }
            for(int i = 0; i < 1; i++){
                hardware.mag.pushInRings();
                sleep(250);// tune time
                hardware.mag.setRingPusherResting();
                sleep(150);// tune time
            }
            hardware.mag.collectRings();
            wiggleTheMag.stopRequested = true;
            autoAim.stopRequested = true;
            hardware.turret.maxPositive = prevMaxPositiveTurret;
        }
        if(stack == 2){
            sleep(1250);
            hardware.intake.turnIntake(0);
            if(hardware.mag.currentState == Mag.State.COLLECT) {
                hardware.mag.dropRings();
                sleep(500);//tune timeout//
            }
            for(int i = 0; i < 3; i++){
                hardware.mag.pushInRings();
                sleep(250);// tune time
                hardware.mag.setRingPusherResting();
                sleep(150);// tune time
            }
            hardware.mag.collectRings();
            sleep(250);
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
            sleep(1300);
            wiggleTheMag.stopRequested = true;
            sleep(250);
            hardware.turret.setMagAngle(hardware.mag.magRotationCollectPosition + 180/540);
            hardware.intake.turnIntake(0);
            if(hardware.mag.currentState == Mag.State.COLLECT) {
                hardware.mag.dropRings();
                sleep(500);//tune timeout
            }
            for(int i = 0; i < 3; i++){
                hardware.mag.pushInRings();
                sleep(250);// tune time
                hardware.mag.setRingPusherResting();
                sleep(150);// tune time
            }

            autoAim.stopRequested = true;
            hardware.turret.maxPositive = prevMaxPositiveTurret;
        }
        hardware.wobbler.goToWobblerDropPosition2();
        hardware.drive.followTrajectoryAsync(dropWobbler1);
        while(hardware.drive.isBusy()&&!isStopRequested()){
            sleep(1);
            hardware.turret.setLocalTurretAngle(0);
            hardware.shooter.updatePID = false;
            hardware.shooter.shooterMotor1.setPower(0);
            hardware.shooter.shooterMotor2.setPower(0);
        }
        hardware.wobbler.releaseWobble();
        sleep(500);
        hardware.wobbler.goToClawRestingPos();
        if(stack == 2 || stack == 1) {
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
                if(hardware.time.milliseconds() > collect2ndWobblerStartTime + 500){
                    hardware.wobbler.moveArmToGrabPos();
                }
            }
            if(hardware.time.milliseconds() > collect2ndWobblerStartTime + 1000){
                hardware.wobbler.releaseWobble();
            }
        }
        hardware.drive.turnAsync(MathFunctions.keepAngleWithin180Degrees( -Math.toRadians(179)-hardware.getAngle()));
        while(hardware.drive.isBusy() && !isStopRequested()){
            sleep(1);
        }
        hardware.wobbler.gripWobble();
        sleep(750);
        hardware.wobbler.goToWobblerDropPosition2();

        hardware.drive.followTrajectoryAsync(dropWobbler2);
        while(hardware.drive.isBusy() && !isStopRequested()){
            sleep(1);
        }
        hardware.wobbler.releaseWobble();

        sleep(250);
        hardware.drive.followTrajectoryAsync(park);
        while(hardware.drive.isBusy() && !isStopRequested()){
            sleep(1);
        }

        while(!isStopRequested()){
            telemetry.addLine("X: " + hardware.drive.getPoseEstimate().getX() + ", Y: "+ hardware.drive.getPoseEstimate().getY()+", heading: "+hardware.drive.getPoseEstimate().getHeading());
            telemetry.update();
        }



    }
}

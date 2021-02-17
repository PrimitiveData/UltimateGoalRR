package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.AutoAim;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.CloseTheCamera;
import org.firstinspires.ftc.teamcode.Ramsete.Pose;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraRotation;
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
        double distanceToPickUp= 8;
        double headingToPickUp = Math.toRadians(0);
        Trajectory pickUpRings = hardware.drive.trajectoryBuilder(pickUpRingsPrelude.end())
                .lineToConstantHeading(new Vector2d(pickUpRingsPrelude.end().getX() + distanceToPickUp*Math.cos(headingToPickUp),pickUpRingsPrelude.end().getY() + distanceToPickUp*Math.sin(headingToPickUp)))
                .build();

        double distanceToPickUp2 = 24;
        Trajectory pickUpRings2 = hardware.drive.trajectoryBuilder(pickUpRings.end())
                .lineToConstantHeading(new Vector2d(pickUpRings.end().getX()+distanceToPickUp2*Math.cos(headingToPickUp),pickUpRings.end().getY() + distanceToPickUp2*Math.sin(headingToPickUp)))
                .build();

        hardware.drive.velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));

        Trajectory dropWobbler1 = null;
        if(stack==0) {

        }
        else if(stack == 1){

        }
        else{
            dropWobbler1 = hardware.drive.trajectoryBuilder(pickUpRings2.end())
                    .lineToConstantHeading(new Vector2d(-102,32))
                    .build();
        }
        Trajectory collect2ndWobbler = null;
        if(stack==0) {
            collect2ndWobbler = hardware.drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(10)
                    .forward(5)
                    .build();
        }
        else if(stack == 1){

        }else{

        }

        Trajectory dropWobbler2;
        if(stack==0) {

        }else if(stack==1){

        }else{

        }

        Trajectory park;
        if(stack==0) {

        }
        else if(stack == 2){

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
        hardware.turret.updatePID = true;
        double ps1TurretAngle=-Math.toRadians(176);
        double ps3TurretAngle=-Math.toRadians(180);
        double ps2TurretAngle=-Math.toRadians(184.5);
        hardware.turret.setLocalTurretAngleAuto(ps1TurretAngle);
        hardware.shooter.shooterVeloPID.setState(1500);
        double goToShootPosStartTime = hardware.time.milliseconds();
        hardware.drive.followTrajectoryAsync(goToShootPos);
        while(hardware.drive.isBusy()){
            if(hardware.time.milliseconds() - goToShootPosStartTime > 450 && hardware.time.milliseconds() - goToShootPosStartTime < 550){
                hardware.intake.raiseBumper();
            }
            sleep(1);
        }
        hardware.intake.dropIntake();
        shootIndividualRing(hardware);
        hardware.turret.setLocalTurretAngleAuto(ps2TurretAngle);
        sleep(800);
        shootIndividualRing(hardware);
        hardware.turret.setLocalTurretAngleAuto(ps3TurretAngle);
        sleep(800);
        shootIndividualRing(hardware);
        hardware.mag.collectRings();
        hardware.intake.turnIntake(1);

        double prevMaxPositiveTurret = hardware.turret.maxPositive;
        hardware.turret.maxPositive = Math.toRadians(0);
        AutoAim autoAim = new AutoAim(hardware,telemetry,this);
        autoAim.start();
        hardware.drive.followTrajectoryAsync(pickUpRingsPrelude);
        while(hardware.drive.isBusy()){
            sleep(1);
        }
        hardware.drive.followTrajectoryAsync(pickUpRings);
        while(hardware.drive.isBusy()){
            sleep(1);
        }
        sleep(2000);
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
        hardware.mag.collectRings();
        hardware.intake.turnIntake(1);
        if(stack == 2){
            hardware.drive.followTrajectoryAsync(pickUpRings2);
            while (hardware.drive.isBusy()){
                sleep(1);
            }
            sleep(1500);
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
        }
        autoAim.stopRequested = true;
        hardware.turret.maxPositive = prevMaxPositiveTurret;
        hardware.wobbler.goToWobblerDropPosition();
        hardware.drive.followTrajectoryAsync(dropWobbler1);
        while(hardware.drive.isBusy()){
            sleep(1);
            hardware.turret.setLocalTurretAngle(0);
            hardware.shooter.updatePID = false;
            hardware.shooter.shooterMotor1.setPower(0);
            hardware.shooter.shooterMotor2.setPower(0);
        }
        hardware.wobbler.releaseWobble();
        sleep(500);
        hardware.wobbler.goToClawRestingPos();
        /*
        hardware.turret.turretAngleOffsetAdjustmentConstant = 0;
        hardware.shooter.rampAngleAdjustmentConstant = 0;
        hardware.turret.updatePID = true;
        hardware.turret.turretPID.setState(0);
        hardware.intake.turnIntake(0);

        hardware.turret.updatePID = true;
        hardware.turret.turretPID.setState(0);
        hardware.drive.followTrajectoryAsync(dropWobbler1);
        while(hardware.drive.isBusy()){
            sleep(1);
        }
        hardware.turret.turretPID.setState(0);
        hardware.turret.updatePID = true;
        hardware.turret.turretPID.setState(0);
        hardware.wobbler.goToWobblerDropPosition();
        sleep(600);
        hardware.wobbler.releaseWobble();
        sleep(50);
        hardware.wobbler.goToWobbleStartingPos();
        sleep(50);



        hardware.wobbler.moveArmToGrabPos();

        hardware.drive.followTrajectoryAsync(collect2ndWobbler);
        while(hardware.drive.isBusy()){
            sleep(1);
        }
        hardware.wobbler.gripWobble();
        sleep(250);

        if(stack == 1 || stack == 2){
            hardware.wobbler.raiseWobble();
            sleep(200);
        }
        if(stack == 0){
            hardware.wobbler.raiseWobble();
            sleep(650);
        }
        if(stack == 0){
            hardware.drive.turnAsync(-Math.toRadians(361));
            sleep(3000);

        }else if(stack == 1) {
            hardware.drive.turnAsync(-Math.toRadians(-101));
            sleep(1200);
            hardware.intake.turnIntake(1);
            hardware.turret.updatePID = true;
            hardware.shooter.updatePID = true;
            //goStraightEncoder(0.5,8,hardware);
            hardware.shooter.shooterVeloPID.setState(-1600);
            AutoAim autoAim2 = new AutoAim(hardware,telemetry,this);
            hardware.turret.turretAngleOffsetAdjustmentConstant = Math.toRadians(2);
            autoAim2.start();
            sleep(2000);
            hardware.intake.turnIntake(1);
            sleep(500);
            shootPowershot(hardware);
            autoAim2.stopRequested = true;
            hardware.turret.updatePID=false;

            hardware.shooter.updatePID=false;
            hardware.drive.turnAsync(Math.toRadians(14.5));
            sleep(2000);
        }else{
            hardware.drive.turnAsync(Math.toRadians(-104));
            sleep(1250);
            hardware.intake.turnIntake(1);
            hardware.turret.updatePID = true;
            hardware.shooter.updatePID = true;
            hardware.turret.turretPID.setState(Math.toRadians(-40));
            //goStraightEncoder(0.5,6.5,hardware);
            sleep(1000);
            //goStraightEncoder(0.5,1.25,hardware);
            sleep(1000);
            //goStraightEncoder(0.5,1.35,hardware);
            hardware.shooter.shooterVeloPID.setState(-1600);
            AutoAim autoAim = new AutoAim(hardware,telemetry,this);
            autoAim.start();
            hardware.shooter.rampAngleAdjustmentConstant = -0.02;
            sleep(2650);
            hardware.intake.turnIntake(0);
            shootPowershot(hardware);
            autoAim.stopRequested = true;
            hardware.turret.updatePID=false;
            hardware.turret.setTurretMotorPower(0);
            hardware.drive.turnAsync(Math.toRadians(-3.25));
            sleep(1600);
        }

        if(stack==0){
            //drop 2nd wobble
        }
        else if(stack == 1) {
            //drop 2nd wobble
        }
        else{
            //drop 2nd wobble
        }
        if(stack == 0) {
            hardware.wobbler.goToWobblerDropPosition();
        }
        else{
            hardware.wobbler.goToWobblerDropPosition();
        }
        sleep(300);
        if(stack == 0){
            sleep(600);
        }
        hardware.wobbler.releaseWobble();
        sleep(100);
        if(stack == 0){
            sleep(250);
        }
        hardware.wobbler.goToWobbleStartingPos();
        sleep(50);
        if(stack == 0){
            sleep(250);
        }
        hardware.wobbler.goToClawRestingPos();
        if(stack == 0){
            sleep(100);
        }
        if(stack == 0) {
            //park
        }
        else if(stack == 1){
            //park
        }
        else if(stack == 2){
            //park
        }
        while(!isStopRequested()){
            telemetry.addLine("X: " + hardware.getXAbsoluteCenter() + "Y: " + hardware.getYAbsoluteCenter());
            telemetry.update();
        }
*/


    }
}

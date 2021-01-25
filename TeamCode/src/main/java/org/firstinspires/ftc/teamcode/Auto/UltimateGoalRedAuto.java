package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.AutoAim;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.AutoAim2;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.CloseTheCamera;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.MoveArmDownAfterDropping1stWobbler;
import org.firstinspires.ftc.teamcode.Ramsete.PathEngine;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;
import org.firstinspires.ftc.teamcode.hardware.SixWheelDrive;
import org.firstinspires.ftc.teamcode.vision.UltimateGoalReturnPositionPipeline;

@Autonomous(name = "RedAuto", group = "Autonomous")
public class UltimateGoalRedAuto extends AutoMethods {
    int stack = 2;
    OpenCvCamera webcam;
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(hardware, this);
        Trajectory goToShootPos = null;
        Trajectory dropWobbler1 = null;
        if(stack==0) {
                   }
        else if(stack == 1){
                    }
        else{

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
        hardware.loop();

        hardware.mag.setRingPusherResting();
        hardware.mag.currentState = Mag.State.TOP;
        hardware.mag.magServo.servo.setPosition(0.32);
        waitForStart();
        hardware.intake.dropIntake();
        hardwareThreadInterface.start();
        CloseTheCamera closeCamera = new CloseTheCamera(webcam);
        closeCamera.start();
        RobotLog.dd("CAMERACLOSED","camera closed successfully");
        //hardware.turret.turretPID.leewayDistance = Math.toRadians(0.3);
        hardware.turret.turretPID.kD = 0.35;
        hardware.turret.turretPID.kP = 1.15;
        hardware.turret.turretPID.kI = 3.85;
        hardware.shooter.setRampPosition(0);
        hardware.shooter.shooterVeloPID.setState(-1500);
        hardware.shooter.updatePID = true;
        hardware.turret.turretPID.setState(Math.toRadians(-180));
        hardware.turret.updatePID = true;

        AutoAim2 autoAim1 = new AutoAim2(hardware,telemetry,this);
        hardware.turret.turretAngleOffsetAdjustmentConstant = Math.toRadians(5);
        hardware.shooter.rampAngleAdjustmentConstant = 0.06;
        autoAim1.start();
        hardware.drive.followTrajectoryAsync(goToShootPos);
        hardware.mag.currentState = Mag.State.TOP;
        hardware.mag.feedTopRing();
        sleep(250);
        shootPowershot(hardware);
        hardware.shooter.rampAngleAdjustmentConstant = 0.0625;
        sleep(200);
        shootPowershot(hardware);
        hardware.shooter.rampAngleAdjustmentConstant = 0.075;
        sleep(200);
        shootPowershot(hardware);
        autoAim1.stopRequested = true;
        hardware.turret.turretAngleOffsetAdjustmentConstant = 0;
        hardware.shooter.rampAngleAdjustmentConstant = 0;
        hardware.turret.updatePID = true;
        hardware.turret.turretPID.setState(0);
        hardware.intake.turnIntake(0);

        hardware.turret.updatePID = true;
        hardware.turret.turretPID.setState(0);
        hardware.drive.followTrajectoryAsync(dropWobbler1);
        hardware.turret.turretPID.setState(0);
        hardware.turret.updatePID = true;
        hardware.turret.turretPID.setState(0);
        hardware.wobbler.goToAutoWobblerDropPosition();
        sleep(600);
        hardware.wobbler.releaseWobble();
        sleep(50);
        hardware.wobbler.goToWobbleStartingPos();
        sleep(50);



        hardware.wobbler.moveArmToGrabPos();

        hardware.drive.followTrajectoryAsync(collect2ndWobbler);
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
            hardware.mag.currentState = Mag.State.BOTTOM;
            hardware.mag.feedBottomRing();
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
            hardware.mag.currentState = Mag.State.TOP;
            hardware.mag.feedTopRing();
            sleep(200);
            shootPowershot(hardware);
            hardware.shooter.rampAngleAdjustmentConstant = 0;
            sleep(250);
            shootPowershot(hardware);
            sleep(250);
            shootPowershot(hardware);
            autoAim.stopRequested = true;
            hardware.turret.updatePID=false;
            hardware.turret.setAllTurretServoPowers(0);
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
            hardware.wobbler.goToAutoWobblerDropPosition();
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



    }
}

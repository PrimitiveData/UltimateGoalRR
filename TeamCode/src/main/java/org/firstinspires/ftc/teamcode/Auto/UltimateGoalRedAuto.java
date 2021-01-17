package org.firstinspires.ftc.teamcode.Auto;

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
        Hardware hardware = new Hardware(hardwareMap, telemetry);
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(hardware, this);
        PathEngine goToShootPos = new PathEngine(40,5,"//sdcard//FIRST//UGauto//goToShootPos.txt",hardware,this);
        PathEngine dropWobbler1;
        if(stack==0) {
            dropWobbler1=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler1Stack0.txt", hardware, this);
        }
        else if(stack == 1){
            dropWobbler1=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler1Stack1.txt", hardware, this);
        }
        else{
            dropWobbler1=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler1Stack2.txt", hardware, this);
        }
        PathEngine collect2ndWobbler;
        if(stack==0) {
            collect2ndWobbler=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//collect2ndWobblerStack0.txt", hardware, this);
        }
        else if(stack == 1){
            collect2ndWobbler=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//collect2ndWobblerStack1.txt", hardware, this);
        }else{
            collect2ndWobbler=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//collect2ndWobblerStack2.txt", hardware, this);
        }

        PathEngine dropWobbler2;
        if(stack==0) {
            dropWobbler2 = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler2Stack0.txt", hardware, this);
        }else if(stack==1){
            dropWobbler2 = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler2Stack1.txt", hardware, this);
        }else{
            dropWobbler2 = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler2Stack2.txt", hardware, this);
        }

        PathEngine park = null;
        if(stack==0) {
            park = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//parkStack0.txt", hardware, this);
        }
        else if(stack == 2){
            park = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//parkStack2.txt", hardware, this);
        }
        PathEngine intakeStack = null;
        if(stack==2){
            intakeStack = new PathEngine(40,5,"//sdcard//FIRST//UGauto//intakeStackStack2.txt",hardware,this);
        }
        /*
        PathEngine park;
        if(stack == 0){
            park = new PathEngine(40,5,"//sdcard//FIRST//UGauto//parkStack0.txt",hardware,this);
        }else if(stack == 1){
            park = new PathEngine(40,5,"//sdcard//FIRST//UGauto//parkStack1.txt",hardware,this);
        }else{
            park = new PathEngine(40,5,"//sdcard//FIRST//UGauto//parkStack2.txt",hardware,this);
        }*/
        hardware.wobbler.goToWobbleStartingPos();
        hardware.wobbler.gripWobble();
        hardware.loop();
        goToShootPos.init();
        collect2ndWobbler.init();
        dropWobbler1.init();
        dropWobbler2.init();
        if(stack == 0||stack==2) {
            park.init();
        }
        if(stack == 2){
            intakeStack.init();
        }
        hardware.mag.setRingPusherResting();
        hardware.mag.currentState = Mag.State.TOP;
        hardware.mag.magServo.servo.setPosition(0.32);
        waitForStart();
        if(stack == 2||stack==1){
            SixWheelDrive.kP = 0.015;
            SixWheelDrive.kDecel = 0.05;
        }
        hardware.intake.dropIntake();
        hardwareThreadInterface.start();
        CloseTheCamera closeCamera = new CloseTheCamera(webcam);
        closeCamera.start();
        RobotLog.dd("CAMERACLOSED","camera closed successfully");
        //first powershot
        //hardware.turret.turretPID.leewayDistance = Math.toRadians(0.3);
        hardware.turret.turretPID.kD = 0.35;
        hardware.turret.turretPID.kP = 1.15;
        hardware.turret.turretPID.kI = 3.85;
        hardware.shooter.setRampPosition(0);
        //hardware.turret.turretPID.kD = 0.45;
        //hardware.turret.turretPID.kP = 1.4;
        //hardware.turret.turretPID.kI = 4.15;
        hardware.shooter.shooterVeloPID.setState(-1500);
        hardware.shooter.updatePID = true;
        hardware.turret.turretPID.setState(Math.toRadians(-180));
        hardware.turret.updatePID = true;
        goToShootPos.run(hardware.time,50,0.7,false);
        /*
        turnTo(-30, 1000, hardware);
        AutoAim autoAim = new AutoAim(hardware,telemetry,this);
        autoAim.start();
        sleep(1000);
        hardware.mag.currentState = Mag.State.TOP;
        hardware.mag.feedTopRing();
        sleep(200);
        shootPowershot(hardware);
        sleep(250);
        shootPowershot(hardware);
        sleep(250);
        shootPowershot(hardware);
        autoAim.stopRequested = true;
        hardware.turret.updatePID=false;

         */
        /*
        if((hardware.turret.localTurretAngleRadians()) > Math.toRadians(-178) - hardware.angle){
            hardware.turret.turretPID.setState(Math.toRadians(-180) - hardware.angle);
            sleep(400);
        }
         *//*
        turnTo(-5-0.5, 800,hardware);
        shootPowershot(hardware);
        telemetry.addLine("1st powershot");
        telemetry.update();*/
        //hardware.turret.turretPID.setState(Math.toRadians(-184));
        //sleep(1300);
        /*
        if((hardware.turret.localTurretAngleRadians()) < Math.toRadians(-185) - hardware.angle){
            hardware.turret.turretPID.setState(Math.toRadians(-183) - hardware.angle);
            sleep(400);
        }
         */
/*
        turnTo(-9.5, 1000,hardware);
        shootPowershot(hardware);
        hardware.mag.currentState = Mag.State.MID;
        hardware.mag.feedMidRing();
        telemetry.addLine("2nd powershot");
        telemetry.update();*/
        //hardware.turret.turretPID.setState(Math.toRadians(-189));
        //sleep(1300);
        /*
        if((hardware.turret.localTurretAngleRadians()) > Math.toRadians(-188) - hardware.angle){
            hardware.turret.turretPID.setState(Math.toRadians(-190) - hardware.angle);
            sleep(400);
        }
         */
/*
        turnTo(1.5, 1200,hardware);
        hardware.mag.updateStateAndSetPosition();
        sleep(400);
        shootPowershot(hardware);
        telemetry.addLine("3rd powershot");
        telemetry.update();
        hardware.turret.turretPID.kD = 0.45;
        hardware.turret.turretPID.kP = 1.4;
        hardware.turret.turretPID.kI = 4.15;*/
        //2nd powershot
        /*
        hardware.mag.setRingPusherResting();
        hardware.mag.updateStateAndSetPosition();
        turnTo(Math.toRadians(-3),1500,hardware);
        hardware.mag.pushInRings();
        sleep(500);
        //3rd powershot
        hardware.mag.setRingPusherResting();
        hardware.mag.updateStateAndSetPosition();
        turnTo(Math.toRadians(-6),1500,hardware);
        hardware.mag.pushInRings();
        sleep(500);

        turnTo(-90,1500,hardware);


        double[] turretPosition = MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),-4.72974566929,hardware.angle);
        double distanceToGoal = Math.hypot(turretPosition[1]- FieldConstants.highGoalPosition[1],turretPosition[0] - FieldConstants.highGoalPosition[0]);
        double angleToGoal = Math.atan2(FieldConstants.highGoalPosition[1]-turretPosition[1], FieldConstants.highGoalPosition[0]-turretPosition[0]) - hardware.turret.getTurretOffset(distanceToGoal);
        hardware.shooter.autoRampPositionForHighGoal(distanceToGoal);
        hardware.turret.updatePID = true;
        hardware.turret.setTurretAngle(angleToGoal);
        for(int i = 0; i < 2; i++){
            hardware.mag.updateStateAndSetPosition();
            sleep(750);
            hardware.mag.pushInRings();
            sleep(500);
            if(i == 1){
                break;
            }
            hardware.mag.setRingPusherResting();
            sleep(250);
        }*/
        AutoAim2 autoAim1 = new AutoAim2(hardware,telemetry,this);
        hardware.turret.turretAngleOffsetAdjustmentConstant = Math.toRadians(5);
        hardware.shooter.rampAngleAdjustmentConstant = 0.06;
        autoAim1.start();
        turnTo(-45,650,hardware);
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

        if(stack==1) {
            turnTo(-14, 1000, hardware);
        }
        else if(stack == 2){
            //turnTo(-27,1000,hardware);
        }
        else{
            turnTo(-60.945,1250,hardware);
        }
        hardware.turret.updatePID = true;
        hardware.updatePID = true;
        hardware.turret.turretPID.setState(0);
        dropWobbler1.run(hardware.time,20,0.7,false);
        hardware.turret.turretPID.setState(0);
        hardware.turret.updatePID = true;
        hardware.turret.turretPID.setState(0);
        if(stack == 1){
            turnTo(15, 1000, hardware);
        }
        hardware.wobbler.goToAutoWobblerDropPosition();
        sleep(600);
        hardware.wobbler.releaseWobble();
        sleep(50);
        hardware.wobbler.goToWobbleStartingPos();
        sleep(50);

        if(stack==1){
            turnTo(-90,1000,hardware);
        }
        else if(stack == 2){
            turnTo(-180+10.9,1200,hardware);
        }
        else{
            turnTo(-180 + 15.562,1500,hardware);
        }

        if(stack==1) {
            MoveArmDownAfterDropping1stWobbler moveArmDownAfterDropping1stWobbler = new MoveArmDownAfterDropping1stWobbler(hardware, this);
            moveArmDownAfterDropping1stWobbler.start();
        }
        else{
            hardware.wobbler.moveArmToGrabPos();
        }
        if(stack == 0){
            collect2ndWobbler.run(hardware.time,60,0.7,false);
            turnTo(-180 + 15.562,1500,hardware);

        }else if(stack==1){
            collect2ndWobbler.run(hardware.time, 60, 0.7, false);
            turnTo(-179,250,hardware);
        }else{
            collect2ndWobbler.run(hardware.time,40,0.7,false);
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
            turnTo(-361,3000,hardware);
        }else if(stack == 1) {
            //turnTo(-360, 1000, hardware);
            turnTo(-101,1200,hardware);
            hardware.intake.turnIntake(1);
            hardware.turret.updatePID = true;
            hardware.shooter.updatePID = true;
            goStraightEncoder(0.5,8,hardware);
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
            turnTo(14.5,2000,hardware);
        }else{
            turnTo(-104,1250,hardware);

            hardware.intake.turnIntake(1);
            hardware.turret.updatePID = true;
            hardware.shooter.updatePID = true;
            hardware.turret.turretPID.setState(Math.toRadians(-40));
            goStraightEncoder(0.5,6.5,hardware);
            sleep(1000);
            goStraightEncoder(0.5,1.25,hardware);
            sleep(1000);
            goStraightEncoder(0.5,1.35,hardware);
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


            turnTo(-3.25,1600,hardware);
        }

        if(stack==0){
            //dropWobbler2.run(hardware.time,60,0.8,false);
            goStraightEncoder(-0.4, -22, hardware);
        }
        else if(stack == 1) {
            //dropWobbler2.run(hardware.time, 60, 0.8, false);
            goStraightEncoder(-0.7,-5, hardware);
            goStraightEncoder(-1,-22,hardware);
            goStraightEncoder(-0.6,-6, hardware);
            turnTo(20,500,hardware);
        }
        else{
            //dropWobbler2.run(hardware.time,60,0.8,false);
            goStraightEncoder(-1,-49,hardware);
            goStraightEncoder(-0.75,-1,hardware);
            goStraightEncoder(-0.5,-0.5,hardware);
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
            turnTo(-270, 1000, hardware);
            goStraightEncoder(-1,-15,hardware);
            turnTo(-360,1000,hardware);
            goStraightEncoder(-1,-5,hardware);
        }
        else if(stack == 1){
            goStraightEncoder(0.3,4,hardware);
        }
        else if(stack == 2){
            //park.run(hardware.time,40,0.7,true);
            goStraightEncoder(1,6,hardware);
        }
        while(!isStopRequested()){
            telemetry.addLine("X: " + hardware.getXAbsoluteCenter() + "Y: " + hardware.getYAbsoluteCenter());
            telemetry.update();
        }



    }
}

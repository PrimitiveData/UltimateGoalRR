package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.Ramsete.Pose;
import org.firstinspires.ftc.teamcode.Teleop.Multithreads.MagFlickerController;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.WobblerArm;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "aUltimateGoalTeleop",group="TeleOp")
public class UltimateGoalTeleop extends OpMode {
    HardwareMecanum hardware;
    boolean slowMode = false;
    boolean slowModeToggledPrevLoop = false;
    //toggles
    public boolean manuelTurretControl = true;
    boolean manuelTurretControlToggledPrevLoop = false;
    boolean magUpdateStateAndSetPositionPrevLoop = false;
    boolean manuelRampControl = true;
    boolean manuelRampControlTogglePrevLoop = false;
    boolean shooterOn = false;
    boolean shooterOnTogglePrevLoop = false;
    boolean grip = true;
    boolean gripOnToggledPrevLoop = false;
    boolean gripperResting = true;
    boolean armStateToggledPrevLoop = false;
    public double shooterVelo;
    public boolean teleopStopped = false;

    boolean intakeOn = false;
    boolean intakeOnToggledPrevLoop = false;

    boolean dPadUpToggledPrevLoop = false;
    boolean dPadRightToggledPrevLoop = false;
    boolean dPadLeftToggledPrevLoop = false;
    boolean dPadDownToggledPrevLoop = false;
    MagFlickerController magFlickerController;
    boolean firstLoop;
    double startAngle;
    public boolean currentlyIncrementingMagDuringShooting;
    Thread magThread = new MagFlickerController(hardware, this);
    public void init(){
        msStuckDetectLoop = 15000;
        /*if (T265.slamra == null) {
            T265.slamra = new T265Camera(new Transform2d(),T265.ODOMETRY_COVARIANCE, hardwareMap.appContext);
        }*/
        startAngle = Hardware.angleClassVariable;
        telemetry.addData("startAngle",startAngle);
        hardware = new HardwareMecanum(hardwareMap,telemetry);
        hardware.drive.setPoseEstimate(HardwareMecanum.poseStorage);
        slowMode = false;
        shooterVelo = -1600;
        magFlickerController = new MagFlickerController(hardware,this);
        hardware.mag.setRingPusherResting();
        hardware.wobbler.goToClawRestingPos();
        hardware.wobbler.goToArmRestingPos();
        firstLoop = true;
        currentlyIncrementingMagDuringShooting = false;
    }
    public double logistic(double input, double constantB, double constantC){
        return constantB*(1/(1+ Math.pow(Math.E,-constantC*(input-0.6)))) - constantB/2+0.5532;
    }
    public void start(){
        //T265.slamra.start();
        magFlickerController.start();
        magThread.start();
    }
    public void loop(){
        if(gamepad1.left_trigger > 0) {
            if(!slowModeToggledPrevLoop) {
                slowMode = !slowMode;
            }
            slowModeToggledPrevLoop = true;
        }
        else{
            if(slowModeToggledPrevLoop){
                slowModeToggledPrevLoop = false;
            }
        }
        if(!slowMode) {
            double leftYAbs = Math.abs(gamepad1.left_stick_y);
            double leftXAbs = Math.abs(gamepad1.left_stick_x);
            double rightXAbs = Math.abs(gamepad1.right_stick_x);

            double leftYWeighted =  logistic(leftYAbs, 1, 7.2) * -gamepad1.left_stick_y / leftYAbs;
            double leftXWeighted = logistic(leftXAbs, 1, 7.2) * -gamepad1.left_stick_x / leftXAbs;
            double rightXWeighted = logistic(rightXAbs, 1, 7.2) * -gamepad1.right_stick_x / rightXAbs;

            hardware.drive.setWeightedDrivePower(new Pose2d(-leftYWeighted, -leftXWeighted, -rightXWeighted));
        }
        else{
            double leftYAbs = Math.abs(gamepad1.left_stick_y);
            double leftXAbs = Math.abs(gamepad1.left_stick_x);
            double rightXAbs = Math.abs(gamepad1.right_stick_x);

            double leftYWeighted =  logistic(leftYAbs, 1, 7.2) * -gamepad1.left_stick_y / leftYAbs;
            double leftXWeighted = logistic(leftXAbs, 1, 7.2) * -gamepad1.left_stick_x / leftXAbs;
            double rightXWeighted = logistic(rightXAbs, 1, 7.2) * -gamepad1.right_stick_x / rightXAbs;

            hardware.drive.setWeightedDrivePower(new Pose2d(-leftYWeighted * 0.3, -leftXWeighted * 0.3, -rightXWeighted * 0.3));
        }
        hardware.loop();

        //intake dropper
        if(gamepad2.left_trigger > 0){
            hardware.intake.dropIntake();
        }
        //manuel turret control toggle & turret control
        if(gamepad2.a) {
            if(!manuelTurretControlToggledPrevLoop) {
                manuelTurretControl = !manuelTurretControl;
            }
            manuelTurretControlToggledPrevLoop = true;
        }
        else{
            if(manuelTurretControlToggledPrevLoop){
                manuelTurretControlToggledPrevLoop = false;
            }
        }
        if(manuelTurretControl){
            hardware.turret.updatePID = false;
            telemetry.addLine("manuel turret control on rn");
            hardware.turret.setTurretMotorPower(gamepad2.left_stick_x);
        }
        else{
            hardware.turret.updatePID = true;
            hardware.turret.pointTowardsHighGoal();
        }
        telemetry.addData("turret Position",hardware.turret.turretMotor.getCurrentPosition());
        //intake control
        if(gamepad1.right_trigger>0) {
            if(!intakeOnToggledPrevLoop) {
                intakeOn = !intakeOn;
            }
            intakeOnToggledPrevLoop = true;
        }
        else{
            if(intakeOnToggledPrevLoop){
                intakeOnToggledPrevLoop = false;
            }
        }
        if(intakeOn) {
            if(gamepad1.a){
                hardware.intake.turnIntake(-1);
            }else {
                hardware.intake.turnIntake(1);
            }
        }
        else{
            hardware.intake.turnIntake(0);
        }
        //mag control
        if(gamepad1.right_bumper) {
            if(!magUpdateStateAndSetPositionPrevLoop) {
                magFlickerController.shootAllRings();
            }
            magUpdateStateAndSetPositionPrevLoop = true;
        }
        else{
            if(magUpdateStateAndSetPositionPrevLoop){
                magUpdateStateAndSetPositionPrevLoop = false;
            }
        }
        //ramp manuel control and automatic control
        if(gamepad1.x) {
            if(!manuelRampControlTogglePrevLoop) {
                manuelRampControl = !manuelRampControl;
                if(manuelRampControl){
                    hardware.turret.updatePID = false;
                }
            }
            manuelRampControlTogglePrevLoop = true;
        }
        else{
            if(manuelRampControlTogglePrevLoop){
                manuelRampControlTogglePrevLoop = false;
            }
        }
        if(manuelRampControl){
            hardware.shooter.setRampPosition(hardware.shooter.rampPostion - gamepad2.right_stick_y*0.001);
            hardware.turret.setMagAngle(0.5);
        }
        else{
            double[] turretPosition = MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),-4.72974566929,hardware.getAngle());
            telemetry.addLine("Turret Position: "+turretPosition[0]+", "+turretPosition[1]);
            double distanceToGoal = Math.hypot(turretPosition[1]- FieldConstants.highGoalPosition[1],turretPosition[0] - FieldConstants.highGoalPosition[0]);
            double angleToGoal = Math.atan2(FieldConstants.highGoalPosition[1]-turretPosition[1], FieldConstants.highGoalPosition[0]-turretPosition[0]) + hardware.turret.getTurretOffset(distanceToGoal);
            telemetry.addData("angleToGoal", Math.toDegrees(angleToGoal));
            if(!currentlyIncrementingMagDuringShooting) {
                hardware.shooter.autoRampPositionForHighGoal(distanceToGoal);
            }
            hardware.turret.updatePID = true;
            hardware.turret.setTurretAngle(angleToGoal);
            shooterVelo = hardware.shooter.autoaimShooterSpeed(distanceToGoal);
            /*if(gamepad2.dpad_down){
                hardware.shooter.rampAngleAdjustmentConstant -= 0.001;
            }
            if(gamepad2.dpad_up){
                hardware.shooter.rampAngleAdjustmentConstant += 0.001;
            }
            if(gamepad2.dpad_left){
                hardware.turret.turretAngleOffsetAdjustmentConstant += Math.toDegrees(0.01);
            }
            if(gamepad2.dpad_right){
                hardware.turret.turretAngleOffsetAdjustmentConstant -= Math.toDegrees(0.01);
            }*/
        }
        //shooter
        if(gamepad1.left_bumper) {
            if(!shooterOnTogglePrevLoop) {
                shooterOn = !shooterOn;
                if(shooterOn){
                    hardware.shooter.firstUpdateShooterPIDFLoop = true;
                }
            }
            shooterOnTogglePrevLoop = true;
            hardware.shooter.shooterVeloPID.clearI();
        }
        else{
            if(shooterOnTogglePrevLoop){
                shooterOnTogglePrevLoop = false;
            }
        }
        if(shooterOn){
            hardware.shooter.updatePID = true;
            /*double voltage = VelocityPIDDrivetrain.getBatteryVoltage();
            double maxVolts = -10.5;
            hardware.shooter.shooterMotor2.setPower(maxVolts/voltage);
            hardware.shooter.shooterMotor1.setPower(maxVolts/voltage);*/
            hardware.shooter.shooterVeloPID.setState(shooterVelo);
        }
        else{
            hardware.shooter.updatePID = false;
            hardware.shooter.shooterMotor2.setPower(-0.5);
            hardware.shooter.shooterMotor1.setPower(-0.5);
        }
        //Flicker
        if(gamepad2.x){
            hardware.mag.pushInRings();
        }
        if(gamepad2.y){
            hardware.mag.setRingPusherResting();
        }
        //wobbler
        if(gamepad2.left_bumper) {
            if(!gripOnToggledPrevLoop) {
                grip = !grip;
                gripperResting = false;
            }
            gripOnToggledPrevLoop = true;
        }
        else{
            if(gripOnToggledPrevLoop){
                gripOnToggledPrevLoop = false;
            }
        }
        if(gripperResting){
            hardware.wobbler.goToClawRestingPos();
        }
        else if(grip){
            hardware.wobbler.gripWobble();
        }else{
            hardware.wobbler.releaseWobble();
        }
        if(gamepad2.right_bumper) {
            if(!armStateToggledPrevLoop) {
                hardware.wobbler.toggleArmState();
            }
            armStateToggledPrevLoop = true;
        }
        else{
            if(armStateToggledPrevLoop){
                armStateToggledPrevLoop = false;
            }
        }
        if(gamepad2.right_trigger>0){
            hardware.wobbler.armState = WobblerArm.ArmState.START;
            hardware.wobbler.goToArmRestingPos();
        }
        //end powershot

        if(gamepad1.dpad_left){
            HardwareThreadInterface hardwareThreadInterface = new HardwareThreadInterface(hardware,this);
            hardwareThreadInterface.start();
            hardware.shooter.shooterVeloPID.setState(-1300);
            hardware.shooter.setRampPosition(0);
            hardware.shooter.updatePID = true;
            hardware.turret.updatePID = true;
            hardware.turret.updatePID = true;
            hardware.turret.turretPID.setState(0);
            hardware.turret.updatePID = true;
            double startingAngle = hardware.getAngle();
            sleeep(1500);
            double powershot1angle = MathFunctions.keepAngleWithinSetRange(startingAngle - Math.toRadians(180), startingAngle + Math.toRadians(180), Math.toRadians(2)+startingAngle);
            double powershot2angle = MathFunctions.keepAngleWithinSetRange(startingAngle - Math.toRadians(180), startingAngle + Math.toRadians(180), Math.toRadians(-5.5)+startingAngle);
            double powershot3angle = MathFunctions.keepAngleWithinSetRange(startingAngle - Math.toRadians(180), startingAngle + Math.toRadians(180), Math.toRadians(-10)+startingAngle);
            hardware.mag.dropRings();
            hardware.drive.turnAsync(powershot1angle);
            sleeep(1000);
            shootRing(hardware);
            hardware.drive.turnAsync(powershot2angle);
            sleeep(1500);
            shootRing(hardware);
            hardware.drive.turnAsync(powershot3angle);
            sleeep(1750);
            shootRing(hardware);
            hardware.turret.updatePID = false;
            hardware.shooter.shooterVeloPID.setState(-1600);
            hardware.shooter.updatePID = false;
            hardwareThreadInterface.stopLooping = true;
        }

        if(gamepad1.dpad_up){
            hardware.drive.setPoseEstimate(new Pose2d(-3.25,-3,0));
        }
        if(gamepad1.dpad_down){
            hardware.drive.setPoseEstimate(new Pose2d(-3.25,20.75,0));
        }
        /*if(gamepad1.dpad_right){
            hardware.shooter.setRampPosition(0);
            hardware.mag.updateStateAndSetPosition();
            sleeep(500);
            hardware.mag.pushInRings();
            sleeep(250);
            hardware.mag.setRingPusherResting();
        }*/
        telemetry.addData("shooter On",shooterOn);
        telemetry.addData("Wobbler grip",grip);
        telemetry.addData("Flap position",hardware.shooter.rampPostion);
        telemetry.addLine("angle: "+hardware.getAngle() + ", in degrees: "+ Math.toDegrees(hardware.getAngle()));
        telemetry.addLine("XCenter: " + hardware.getXAbsoluteCenter()  + ", YCenter: "+hardware.getYAbsoluteCenter());
        telemetry.addLine("left position: " + -hardware.hub1Motors[0].getCurrentPosition() + ", right position: " + -hardware.hub1Motors[3].motor.getCurrentPosition() + ", lateral position: " + hardware.hub1Motors[1].getCurrentPosition());
        telemetry.addLine("shooter velo: "+shooterVelo);
        //telemetry.addLine("turret Angle: "+ Math.toDegrees(hardware.turret.localTurretAngleRadians())+", turret output power: "+gamepad2.left_stick_x);
        telemetry.update();
//correcting autoaim
        if(gamepad2.dpad_up) {
            if(!dPadUpToggledPrevLoop) {
                hardware.shooter.rampAngleAdjustmentConstant += 0.05;
            }
            dPadUpToggledPrevLoop = true;
        }
        else{
            if(dPadUpToggledPrevLoop){
                dPadUpToggledPrevLoop = false;
            }
        }
        if(gamepad2.dpad_down) {
            if(!dPadDownToggledPrevLoop) {
                hardware.shooter.rampAngleAdjustmentConstant -= 0.05;
            }
            dPadDownToggledPrevLoop = true;
        }
        else{
            if(dPadDownToggledPrevLoop){
                dPadDownToggledPrevLoop = false;
            }
        }
        if(gamepad2.dpad_left) {
            if(!dPadLeftToggledPrevLoop) {
                hardware.turret.turretAngleOffsetAdjustmentConstant += Math.toRadians(1);
            }
            dPadLeftToggledPrevLoop = true;
        }
        else{
            if(dPadLeftToggledPrevLoop){    
                dPadLeftToggledPrevLoop = false;
            }
        }
        if(gamepad2.dpad_right) {
            if(!dPadRightToggledPrevLoop) {
                hardware.turret.turretAngleOffsetAdjustmentConstant -= Math.toRadians(1);
            }
            dPadRightToggledPrevLoop = true;
        }
        else{
            if(dPadRightToggledPrevLoop){
                dPadRightToggledPrevLoop = false;
            }
        }

    }
    public void stop(){
        //T265.slamra.stop();
        teleopStopped = true;
        try {
            magFlickerController.writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void shootRing(HardwareMecanum hardware) {
        hardware.mag.pushInRings();
        sleeep(250);
        hardware.mag.setRingPusherResting();
    }


    public void sleeepInterrupt(double milliseconds){
        double startTime = hardware.time.milliseconds();
        while(hardware.time.milliseconds() < startTime + milliseconds && !teleopStopped && !gamepad2.b){
            try{
                Thread.sleep(10);
            }catch(InterruptedException e){

            }
        }
    }

    public void sleeep(double milliseconds){
        double startTime = hardware.time.milliseconds();
        while(hardware.time.milliseconds() < startTime + milliseconds && !teleopStopped){
            try{
                Thread.sleep(10);
            }catch(InterruptedException e){

            }
        }
    }
}

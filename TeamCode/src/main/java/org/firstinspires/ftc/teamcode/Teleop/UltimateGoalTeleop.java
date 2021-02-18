package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.Ramsete.Pose;
import org.firstinspires.ftc.teamcode.Teleop.Multithreads.MagFlickerController;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
    //toggles
    boolean magUpdateStateAndSetPositionPrevLoop = false;
    boolean bumperDown;
    boolean bumperPrevLoop = false;
    int magTrigger = 0;
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

    boolean toggleMagStatePrevLoop = false;

    MagFlickerController magFlickerController;
    boolean firstLoop;
    double startAngle;
    public boolean currentlyIncrementingMagDuringShooting;

    enum Mode {DRIVER_CONTROL, ALIGN_TO_POINT}
    private Mode driveMode;
    private PIDFController headingController;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public void init(){
        msStuckDetectLoop = 15000;
        /*if (T265.slamra == null) {
            T265.slamra = new T265Camera(new Transform2d(),T265.ODOMETRY_COVARIANCE, hardwareMap.appContext);
        }*/
        startAngle = Hardware.angleClassVariable;
        telemetry.addData("startAngle",startAngle);
        hardware = new HardwareMecanum(hardwareMap,telemetry);
        hardware.drive.setPoseEstimate(HardwareMecanum.poseStorage);
        hardware.cumulativeAngle = HardwareMecanum.cumulativeAngleStorage;
        hardware.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterVelo = 1500;
        magFlickerController = new MagFlickerController(hardware,this);
        hardware.mag.setRingPusherResting();
        hardware.wobbler.goToClawRestingPos();
        hardware.wobbler.goToArmRestingPos();
        firstLoop = true;
        currentlyIncrementingMagDuringShooting = false;
        driveMode = Mode.DRIVER_CONTROL;
        headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
        hardware.turret.turretMotor.readRequested = true;
        bumperDown = false;
    }
    public double logistic(double input, double constantB, double constantC){
        return constantB*(1/(1+ Math.pow(Math.E,-constantC*(input-0.6)))) - constantB/2+0.5532;
    }
    public void start(){
        //T265.slamra.start();
        magFlickerController.start();
        hardware.mag.collectRings();
    }
    public void loop(){
        switch(driveMode){
            case DRIVER_CONTROL:
                if(gamepad1.b)
                    driveMode = Mode.ALIGN_TO_POINT;
                hardware.updateDrivePID = false;
                //driver contro
                    /*
                    Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-hardware.getAngle());
                    double leftYAbs = Math.abs(input.getY());
                    double leftXAbs = Math.abs(input.getX());
                    double rightXAbs = Math.abs(gamepad1.right_stick_x);
                    */
                    double leftYAbs = gamepad1.left_stick_y;
                    double leftXAbs = gamepad1.left_stick_x;
                    double rightXAbs = gamepad1.right_stick_x;

                    // for field centric references to raw gamepad left stick inputs must be changed to the rotated values
                    //double leftYWeighted =  logistic(leftYAbs, 1, 7.2) * -gamepad1.left_stick_y / leftYAbs;
                    //double leftXWeighted = logistic(leftXAbs, 1, 7.2) * -gamepad1.left_stick_x / leftXAbs;
                    //double rightXWeighted = logistic(rightXAbs, 1, 7.2) * -gamepad1.right_stick_x / rightXAbs;
                    hardware.drive.setWeightedDrivePower(new Pose2d(-leftYAbs, -leftXAbs, -rightXAbs));
                    //hardware.drive.setWeightedDrivePower(new Pose2d(-leftYWeighted * 0.3, -leftXWeighted * 0.3, -rightXWeighted * 0.3));
                break;
            case ALIGN_TO_POINT:
                if(gamepad1.y)
                    driveMode = Mode.DRIVER_CONTROL;
                hardware.updateDrivePID = true;
        }
        hardware.loop();

        //intake dropper
        if(gamepad2.left_trigger > 0){
            hardware.intake.dropIntake();
        }
        //manuel turret control toggle & turret control

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
        //bumper control
        if(gamepad1.right_bumper){
            if(!bumperPrevLoop) {
                bumperDown = !bumperDown;
            }
            bumperPrevLoop = true;
        }
        else{
            if(bumperPrevLoop){
                bumperPrevLoop = false;
            }
        }
        if(bumperDown){
            hardware.intake.dropIntake();
        }else{
            hardware.intake.raiseBumper();
        }
        if(gamepad1.left_trigger > 0) {
            if(!magUpdateStateAndSetPositionPrevLoop) {
                magTrigger++;
                if(magTrigger == 2) {
                    magFlickerController.shootAllRings();
                    magTrigger = 0;
                }
                else
                    hardware.mag.dropRings();
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
        }
        else{
            double[] turretPosition = MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),-4.72974566929,hardware.getAngle());
            telemetry.addLine("Turret XY Position On Field: "+turretPosition[0]+", "+turretPosition[1]);
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
            hardware.shooter.shooterVeloPID.setState(1500);
            /*double voltage = VelocityPIDDrivetrain.getBatteryVoltage();
            double maxVolts = -10.5;
            hardware.shooter.shooterMotor2.setPower(maxVolts/voltage);
            hardware.shooter.shooterMotor1.setPower(maxVolts/voltage);*/
        }
        else{
            hardware.shooter.updatePID = false;
            hardware.shooter.shooterMotor2.setPower(0);
            hardware.shooter.shooterMotor1.setPower(0);
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


        /*
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
        if(gamepad2.x) {
            if(!toggleMagStatePrevLoop) {
                hardware.mag.toggleStates();
            }
            toggleMagStatePrevLoop = true;
        }
        else{
            if(toggleMagStatePrevLoop){
                toggleMagStatePrevLoop = false;
            }
        }
        telemetry.addData("Shooter Velocity: ",shooterVelo);
        telemetry.addData("Local Turret Angle: ", Math.toDegrees(hardware.turret.localTurretAngleRadians()));
        telemetry.addData("Shooter On: ",shooterOn);
        telemetry.addLine("Robot Angle: " + Math.toDegrees(hardware.getAngle()));
        telemetry.addLine("XCenter: " + hardware.getXAbsoluteCenter()  + ", YCenter: "+hardware.getYAbsoluteCenter());
        telemetry.addLine("Left Odo Pos: " + -hardware.hub1Motors[0].getCurrentPosition() + ", Right Odo Pos: " + -hardware.hub1Motors[3].motor.getCurrentPosition() + ", Lateral Odo Pos: " + hardware.hub1Motors[1].getCurrentPosition());
        packet.put("targetVelocity: ", hardware.shooter.shooterVeloPID.desiredState);
        packet.put("currentVelocity: ", hardware.shooter.shooterVeloPID.currentState);
        packet.put("Velocity Difference: ", hardware.shooter.shooterVeloPID.desiredState - hardware.shooter.shooterVeloPID.currentState);
        dashboard.sendTelemetryPacket(packet);
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

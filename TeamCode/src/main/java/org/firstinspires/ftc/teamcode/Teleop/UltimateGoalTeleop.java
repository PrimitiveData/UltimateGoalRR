package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.Ramsete.Pose;
import org.firstinspires.ftc.teamcode.Teleop.Multithreads.MagFlickerController;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.PowershotAutoShootInfo;
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
    boolean powershotAutoShoot = false;
    boolean powershotAutoShootPrevLoop = false;
    boolean powershotSequenceShoot = false;
    boolean shooterOn = false;
    boolean shooterOnTogglePrevLoop = false;
    boolean grip = true;
    boolean gripOnToggledPrevLoop = false;
    boolean gripperResting = true;
    boolean armStateToggledPrevLoop = false;
    public double shooterVelo;
    public boolean teleopStopped = false;

    public boolean startMagRotation = false;

    boolean intakeOn = false;
    boolean intakeOnToggledPrevLoop = false;
    boolean intakeOnToggledPrevLoop2 = false;

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
    public Mode driveMode;
    public PIDFController headingController;

    double fakeOdoY;
    double odoOffset = 0;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    boolean slowMode;
    boolean slowModeToggledPrevLoop;

    public void init(){
        msStuckDetectLoop = 30000;
        /*if (T265.slamra == null) {
            T265.slamra = new T265Camera(new Transform2d(),T265.ODOMETRY_COVARIANCE, hardwareMap.appContext);
        }*/
        startAngle = Hardware.angleClassVariable;
        telemetry.addData("startAngle",startAngle);
        hardware = new HardwareMecanum(hardwareMap,telemetry,false);
        hardware.drive.setPoseEstimate(HardwareMecanum.poseStorage);
        hardware.cumulativeAngle = HardwareMecanum.cumulativeAngleStorage;
        hardware.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterVelo = 1350;
        magFlickerController = new MagFlickerController(hardware,this);
        hardware.mag.setRingPusherResting();
        hardware.wobbler.raiseWobble();
        hardware.wobbler.goToClawRestingPos();
        firstLoop = true;
        currentlyIncrementingMagDuringShooting = false;
        driveMode = Mode.DRIVER_CONTROL;
        headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
        hardware.turret.turretMotor.readRequested = true;
        bumperDown = false;
        hardware.loop();
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

        if(gamepad1.dpad_right) {
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
        if(slowMode){
            hardware.drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*0.3, -gamepad1.left_stick_x *0.3, - gamepad1.right_stick_x * 0.3));
        }
        else {
            hardware.drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            telemetry.addData("Left Y Input: ", -gamepad1.left_stick_y);
            telemetry.addData("Left X Input: ", -gamepad1.left_stick_x);
            telemetry.addData("Right X Input: ", -gamepad1.right_stick_x);
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
                if(intakeOn) {
                    hardware.mag.collectRings();
                }
            }
            intakeOnToggledPrevLoop = true;
        }
        else{
            if(intakeOnToggledPrevLoop){
                intakeOnToggledPrevLoop = false;
            }
        }

        if(gamepad1.b){
            if(!intakeOnToggledPrevLoop2) {
                hardware.intake.crServoOn = !hardware.intake.crServoOn;
            }
            intakeOnToggledPrevLoop2 = true;
        }
        else{
            if(intakeOnToggledPrevLoop2){
                intakeOnToggledPrevLoop2 = false;
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
                else {
                    startMagRotation = true;
                }
            }
            magUpdateStateAndSetPositionPrevLoop = true;
        }
        else{
            if(magUpdateStateAndSetPositionPrevLoop){
                magUpdateStateAndSetPositionPrevLoop = false;
            }
        }

        if(gamepad2.y){
            magFlickerController.shootSingleRing();
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
            hardware.turret.setTurretMotorPower(0);
            hardware.shooter.setRampPosition(hardware.shooter.rampPostion - gamepad2.right_stick_y*0.001);
        }
        else{
            double[] turretPosition = MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),-4.22,hardware.getAngle());
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
            hardware.shooter.shooterVeloPID.setState(shooterVelo);
            /*double voltage = VelocityPIDDrivetrain.getBatteryVoltage();
            double maxVolts = -10.5;
            hardware.shooter.shooterMotor2.setPower(maxVolts/voltage);
            hardware.shooter.shooterMotor1.setPower(maxVolts/voltage);*/
        }
        else{
            hardware.shooter.updatePID = false;
            hardware.shooter.shooterMotor2.setPower(0);
            hardware.shooter.shooterMotor1.setPower(0);
            hardware.shooter.shooterVeloPID.setState(0);
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
        if(gamepad2.a){

            HardwareThreadInterface hardwareThreadInterface = new HardwareThreadInterface(hardware,this);
            hardwareThreadInterface.start();
            hardware.shooter.setRampPosition(0.2);
            hardware.turret.updatePID = true;
            hardware.shooter.updatePID = true;
            hardware.shooter.shooterVeloPID.setState(1350);

            if(hardware.mag.currentState == Mag.State.COLLECT){
                hardware.mag.dropRings();
                sleeep(500);
            }

            hardware.mag.dropRings();

            double ps1TurretAngle=Math.toRadians(3.25);
            double ps2TurretAngle=Math.toRadians(-2.75);
            double ps3TurretAngle=Math.toRadians(-9.25);
            ElapsedTime powershotTimer = new ElapsedTime();

            for(int i = 0; i < 3; i++) {
                double powershotAngleCurrent;
                if(i == 0){
                    powershotAngleCurrent = ps1TurretAngle;
                }else if(i == 1){
                    powershotAngleCurrent = ps2TurretAngle;
                }else{
                    powershotAngleCurrent = ps3TurretAngle;
                }
                double prevTurretAngle = hardware.turret.localTurretAngleRadians();
                hardware.turret.setLocalTurretAngle(powershotAngleCurrent);
                while (!teleopStopped) {
                    double currentTurretAngle = hardware.turret.localTurretAngleRadians();
                    if (Math.abs(currentTurretAngle - prevTurretAngle) > Math.toRadians(0.1))
                        powershotTimer.reset();
                    if (powershotTimer.milliseconds() >= 200 && Math.abs(currentTurretAngle - powershotAngleCurrent) < Math.toRadians(0.1) && -hardware.shooter.shooterMotor1.getVelocity() > 1300)
                        break;
                    prevTurretAngle = currentTurretAngle;
                }
                hardware.mag.pushInRings();
                sleeep(200);// tune time
                hardware.mag.setRingPusherResting();
                sleeep(150);
            }
            hardware.mag.collectRings();
            hardwareThreadInterface.stopLooping = true;
            sleeep(50);
        }
        /*
        if(gamepad2.a){
            hardware.drive.setPoseEstimate(new Pose2d(-59,-32.25,Math.toRadians(180)));
            hardware.cumulativeAngle = Math.toRadians(180);
            hardware.prevAngle = Math.toRadians(180);
            HardwareThreadInterface hardwareThreadInterface = new HardwareThreadInterface(hardware,this);
            hardwareThreadInterface.start();
            hardware.shooter.setRampPosition(0.19);
            hardware.turret.updatePID = true;
            hardware.shooter.updatePID = true;
            hardware.shooter.shooterVeloPID.setState(1300);
            if(hardware.mag.currentState == Mag.State.COLLECT){
                hardware.mag.dropRings();
                sleeep(500);
            }
            hardware.mag.dropRings();
            Trajectory goToPowershotPos = hardware.drive.trajectoryBuilder(hardware.drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-59,-2.75))
                    .build();
            hardware.drive.followTrajectoryAsync(goToPowershotPos);
            while(hardware.drive.isBusy()){
                sleeep(1);
            }
            hardware.drive.turnAsync(MathFunctions.keepAngleWithin180Degrees(Math.toRadians(180)-hardware.getAngle()));
            while(hardware.drive.isBusy()){
                sleeep(1);
            }
            double ps1TurretAngle=Math.toRadians(3)-hardware.getAngle() + Math.toRadians(180);
            double ps2TurretAngle=Math.toRadians(-4)-hardware.getAngle() + Math.toRadians(180);
            double ps3TurretAngle=Math.toRadians(-10)-hardware.getAngle() + Math.toRadians(180);
            ElapsedTime powershotTimer = new ElapsedTime();
            for(int i = 0; i < 3; i++) {
                double powershotAngleCurrent;
                if(i == 0){
                    powershotAngleCurrent = ps1TurretAngle;
                }else if(i == 1){
                    powershotAngleCurrent = ps2TurretAngle;
                }else{
                    powershotAngleCurrent = ps3TurretAngle;
                }
                double prevTurretAngle = hardware.turret.localTurretAngleRadians();
                hardware.turret.setLocalTurretAngle(powershotAngleCurrent);
                while (!teleopStopped) {
                    double currentTurretAngle = hardware.turret.localTurretAngleRadians();
                    if (Math.abs(currentTurretAngle - prevTurretAngle) > Math.toRadians(0.8))
                        powershotTimer.reset();
                    if (powershotTimer.milliseconds() >= 200 && Math.abs(currentTurretAngle - powershotAngleCurrent) < Math.toRadians(0.25) && -hardware.shooter.shooterMotor1.getVelocity() > 1250)
                        break;
                    prevTurretAngle = currentTurretAngle;
                }
                hardware.mag.pushInRings();
                sleeep(500);// tune time
                hardware.mag.setRingPusherResting();
                sleeep(150);
            }
            hardwareThreadInterface.stopLooping = true;
            sleeep(50);
        }
        if(gamepad1.dpad_up){
            if(MathFunctions.keepAngleWithin180Degrees(hardware.getAngle()) < Math.toRadians(-90) || MathFunctions.keepAngleWithin180Degrees(hardware.getAngle())> Math.toRadians(90)) {
                hardware.drive.setPoseEstimate(new Pose2d(hardware.getXAbsoluteCenter(), -32.25, Math.toRadians(180)));
                hardware.cumulativeAngle = Math.toRadians(180);
                hardware.prevAngle = Math.toRadians(180);
            }else{
                hardware.drive.setPoseEstimate(new Pose2d(hardware.getXAbsoluteCenter(),-32.25,0));
                hardware.cumulativeAngle = 0;
                hardware.prevAngle = 0;
            }
        }
        if(gamepad1.dpad_down){
            if(MathFunctions.keepAngleWithin180Degrees(hardware.getAngle()) < Math.toRadians(-90) || MathFunctions.keepAngleWithin180Degrees(hardware.getAngle())> Math.toRadians(90)) {
                hardware.drive.setPoseEstimate(new Pose2d(hardware.getXAbsoluteCenter(), 45.25, Math.toRadians(180)));
                hardware.cumulativeAngle = Math.toRadians(180);
                hardware.prevAngle = Math.toRadians(180);
            }else{
                hardware.drive.setPoseEstimate(new Pose2d(hardware.getXAbsoluteCenter(),45.25,0));
                hardware.cumulativeAngle = 0;
                hardware.prevAngle = 0;
            }
        }
        if(gamepad1.dpad_left){
            if(MathFunctions.keepAngleWithin180Degrees(hardware.getAngle()) < Math.toRadians(-90) || MathFunctions.keepAngleWithin180Degrees(hardware.getAngle())> Math.toRadians(90)) {
                hardware.drive.setPoseEstimate(new Pose2d(1, hardware.getYAbsoluteCenter(), Math.toRadians(180)));
                hardware.cumulativeAngle = Math.toRadians(180);
                hardware.prevAngle = Math.toRadians(180);
            }else{
                hardware.drive.setPoseEstimate(new Pose2d(1,hardware.getYAbsoluteCenter(),0));
                hardware.cumulativeAngle = 0;
                hardware.prevAngle = 0;
            }
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
*/
        /*
        if(gamepad1.dpad_up){
            if(MathFunctions.keepAngleWithin180Degrees(hardware.getAngle()) < Math.toRadians(-90) || MathFunctions.keepAngleWithin180Degrees(hardware.getAngle())> Math.toRadians(90)) {
                hardware.drive.setPoseEstimate(new Pose2d(hardware.getXAbsoluteCenter(), -32.25, Math.toRadians(180)));
                hardware.cumulativeAngle = Math.toRadians(180);
                hardware.prevAngle = Math.toRadians(180);
            }else{
                hardware.drive.setPoseEstimate(new Pose2d(hardware.getXAbsoluteCenter(),-32.25,0));
                hardware.cumulativeAngle = 0;
                hardware.prevAngle = 0;
            }
        }
        if(gamepad1.dpad_down){
            if(MathFunctions.keepAngleWithin180Degrees(hardware.getAngle()) < Math.toRadians(-90) || MathFunctions.keepAngleWithin180Degrees(hardware.getAngle())> Math.toRadians(90)) {
                hardware.drive.setPoseEstimate(new Pose2d(hardware.getXAbsoluteCenter(), 45.25, Math.toRadians(180)));
                hardware.cumulativeAngle = Math.toRadians(180);
                hardware.prevAngle = Math.toRadians(180);
            }else{
                hardware.drive.setPoseEstimate(new Pose2d(hardware.getXAbsoluteCenter(),45.25,0));
                hardware.cumulativeAngle = 0;
                hardware.prevAngle = 0;
            }
        }
        if(gamepad1.dpad_left){
            if(MathFunctions.keepAngleWithin180Degrees(hardware.getAngle()) < Math.toRadians(-90) || MathFunctions.keepAngleWithin180Degrees(hardware.getAngle())> Math.toRadians(90)) {
                hardware.drive.setPoseEstimate(new Pose2d(1, hardware.getYAbsoluteCenter(), Math.toRadians(180)));
                hardware.cumulativeAngle = Math.toRadians(180);
                hardware.prevAngle = Math.toRadians(180);
            }else{
                hardware.drive.setPoseEstimate(new Pose2d(1,hardware.getYAbsoluteCenter(),0));
                hardware.cumulativeAngle = 0;
                hardware.prevAngle = 0;
            }
        }
         */

        if(gamepad1.dpad_up){
            hardware.turret.setLocalTurretAngle(0);
            hardware.turret.updatePID = true;
            hardware.shooter.setRampPosition(0.55);
        }
        if(gamepad1.dpad_left){
            hardware.turret.setLocalTurretAngle(0);
            hardware.turret.updatePID = true;
            hardware.shooter.setRampPosition(0.2);
        }
        if(gamepad1.dpad_down){
            hardware.turret.setLocalTurretAngle(0);
            hardware.turret.updatePID = true;
            hardware.shooter.setRampPosition(0.35);
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
        telemetry.addData("Current Shooter Velocity: ",hardware.shooter.shooterMotor1.getVelocity());
        telemetry.addData("Local Turret Angle: ", Math.toDegrees(hardware.turret.localTurretAngleRadians()));
        telemetry.addData("Shooter On: ",shooterOn);
        telemetry.addLine("Robot Angle: " + Math.toDegrees(hardware.getAngle()));
        telemetry.addLine("XCenter: " + hardware.getXAbsoluteCenter()  + ", YCenter: "+hardware.getYAbsoluteCenter());
        telemetry.addLine("Left Odo Pos: " + -hardware.hub1Motors[0].getCurrentPosition() + ", Right Odo Pos: " + -hardware.hub1Motors[3].motor.getCurrentPosition() + ", Lateral Odo Pos: " + hardware.hub1Motors[1].getCurrentPosition());
        packet.put("targetVelocity: ", hardware.shooter.shooterVeloPID.desiredState);
        packet.put("currentVelocity: ", hardware.shooter.shooterVeloPID.currentState);
        packet.put("Velocity Difference: ", hardware.shooter.shooterVeloPID.desiredState - hardware.shooter.shooterVeloPID.currentState);
        //dashboard.sendTelemetryPacket(packet);
        telemetry.update();
//correcting autoaim
        if(gamepad2.dpad_up) {
            if(!dPadUpToggledPrevLoop) {
                hardware.shooter.rampAngleAdjustmentConstant += 0.025;
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
                hardware.shooter.rampAngleAdjustmentConstant -= 0.025;
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
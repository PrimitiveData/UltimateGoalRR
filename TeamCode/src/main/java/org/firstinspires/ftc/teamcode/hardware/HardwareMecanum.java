package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Intake;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Shooter;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Turret;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.WobblerArm;

import java.util.List;

public class HardwareMecanum {
    public HardwareMap hardwareMap;
    static HardwareMecanum hw;
    public int ticker = 0;
    public Motor[] hub1Motors;
    public Motor[] hub2Motors;
    public RegServo[] servos;
    public ContRotServo[] CRservos;
    public ElapsedTime time;
    public int loops = 0;
    public boolean firstLoop = true;
    public double startTimeHub1;
    public double startTimeHub2;
    public double prevTimeHub1;
    public double prevTimeHub2;
    public double deltaTimeHub1;
    public double deltaTimeHub2;
    public static Telemetry telemetry;
    public Shooter shooter;
    public Turret turret;
    public Intake intake;
    public Mag mag;
    public WobblerArm wobbler;
    public List<LynxModule> allHubs;
    public SampleMecanumDrive drive;
    public Pose2d currentPose;
    public static Pose2d poseStorage;
    public static double cumulativeAngleStorage=0;
    public double cumulativeAngle = 0;
    private double prevAngle;
    public boolean updateDrivePID = false;
    public Pose2d targetPose = new Pose2d(0,0,0);
    public HardwareMecanum(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        hw = this;
        HardwareMecanum.telemetry = telemetry;
        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        if(poseStorage == null){
            currentPose = new Pose2d(0,0,0);
        }else{
            currentPose = poseStorage;
        }
        prevAngle = currentPose.getHeading();
        hub1Motors = new Motor[4];//initialize here
        time = new ElapsedTime();
        hub2Motors = new Motor[4];//initialize here
        servos = new RegServo[12];//initialize here
        CRservos = new ContRotServo[12];
        hub1Motors[0] = new Motor(hardwareMap.get(DcMotorEx.class,"shooterMotor1"));
        hub1Motors[1] = new Motor(hardwareMap.get(DcMotorEx.class,"shooterMotor2"));
        hub1Motors[2] = new Motor(hardwareMap.get(DcMotorEx.class,"intakeMotor"));
        hub1Motors[3] = new Motor(hardwareMap.get(DcMotorEx.class,"turretMotor"));
        servos[0] = new RegServo(hardwareMap.get(Servo.class,"shootAngleController"));
        servos[1] = new RegServo(hardwareMap.get(Servo.class,"intakeDropperGuard"));
        servos[2] = new RegServo(hardwareMap.get(Servo.class,"magServo"));
        servos[4] = new RegServo(hardwareMap.get(Servo.class,"wobblerClaw"));
        servos[5] = new RegServo(hardwareMap.get(Servo.class,"wobblerArm1"));
        servos[6] = new RegServo(hardwareMap.get(Servo.class,"ringPusher"));
        servos[7] = new RegServo(hardwareMap.get(Servo.class, "magRotationServo"));
        servos[8] = new RegServo(hardwareMap.get(Servo.class,"wobblerArm2"));
        CRservos[0] = new ContRotServo(hardwareMap.get(CRServo.class,"intakeFunnelerStarboard"));
        CRservos[1] = new ContRotServo(hardwareMap.get(CRServo.class,"intakeFunnelerPort"));
        shooter = new Shooter(hub1Motors[0],hub1Motors[1],servos[0],this);
        turret = new Turret(hub1Motors[3], servos[7], this);
        intake = new Intake(hub1Motors[2],servos[1],CRservos[0],CRservos[1]);
        mag = new Mag(servos[2],servos[6],this);
        wobbler = new WobblerArm(servos[5],servos[8],servos[4]);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(){
        loops++;
        boolean hub1ReadNeeded = false;
        for(Motor motor: hub1Motors){
            if(motor != null && motor.readRequested){
                hub1ReadNeeded = true;
            }
        }
        if(hub1ReadNeeded) {
            allHubs.get(0).clearBulkCache();
        }
        for(Motor motor: hub1Motors){
            if(motor != null && motor.readRequested){
                motor.currentPosition = motor.motor.getCurrentPosition();
                motor.currentVelocity = motor.motor.getVelocity(AngleUnit.RADIANS);
            }
        }
        if(firstLoop){
            startTimeHub1 = time.milliseconds();
            prevTimeHub1 = startTimeHub1;
        }
        double currentTimeHub1 = time.milliseconds();
        deltaTimeHub1 = currentTimeHub1-prevTimeHub1;
        prevTimeHub1 = currentTimeHub1;
        if(updateDrivePID)
        if(shooter.updatePID) {
            shooter.updateShooterPIDF(deltaTimeHub1 / 1000);
        }
        if(turret.updatePID){
            turret.updateTurretPID();
        }
        for(Motor motor: hub1Motors){
            if(motor!=null&&motor.setTargetPosRequested){
                motor.motor.setTargetPosition(motor.targetPosition);
                motor.setTargetPosRequested = false;
            }
        }
        for(Motor motor: hub1Motors){
            if(motor!=null&&motor.writePowerRequested){
                motor.motor.setPower(motor.power);
                motor.writePowerRequested = false;
            }
            if(motor!=null&&motor.writeVelocityRequested){
                motor.motor.setVelocity(motor.velocity);
                motor.writeVelocityRequested = false;
            }
        }

        boolean hub2ReadNeeded = true;
        if(hub2ReadNeeded) {
            allHubs.get(1).clearBulkCache();
        }
        drive.update();
        if(updateDrivePID) {
            drive.updateDrivetrainPID(drive.getPoseEstimate(), targetPose);
            drive.updateDrivetrainHeadingPID(drive.getPoseEstimate(), targetPose);
        }
        for(int i = 0; i <hub2Motors.length;i++){
            Motor motor = hub2Motors[i];
            if(motor != null && motor.readRequested){
                motor.currentPosition = motor.motor.getCurrentPosition();
                motor.currentVelocity = motor.motor.getVelocity(AngleUnit.RADIANS);
            }
        }
        if(firstLoop){
            startTimeHub2 = time.milliseconds();
            prevTimeHub2 = startTimeHub2;
            firstLoop = false;
        }
        double currentTimeHub2 = time.milliseconds();
        deltaTimeHub2 = currentTimeHub2-prevTimeHub2;
        prevTimeHub2 = currentTimeHub2;
        currentPose = drive.getPoseEstimate();
        poseStorage = currentPose;
        cumulativeAngle += MathFunctions.keepAngleWithin180Degrees(currentPose.getHeading() - prevAngle);
        prevAngle = currentPose.getHeading();
        for(RegServo servo: servos){
            if(servo!=null&&servo.writeRequested){
                servo.servo.setPosition(servo.position);
                servo.writeRequested = false;
            }
        }
        for(ContRotServo CRservo: CRservos){
            if(CRservo!=null&&CRservo.writeRequested) {
                CRservo.servo.setPower(CRservo.power);
                CRservo.writeRequested = false;
            }
        }
    }
    public double getXAbsoluteCenter(){
        return currentPose.getX();
    }
    public double getYAbsoluteCenter(){
        return currentPose.getY();
    }
    public double getAngle(){
        return cumulativeAngle;
    }
    public static HardwareMecanum getInstance(){
        return hw;
    }
}

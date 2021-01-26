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
    public HardwareMecanum(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        hw = this;
        Hardware.telemetry = telemetry;
        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        hub1Motors = new Motor[4];//initialize here
        time = new ElapsedTime();
        hub2Motors = new Motor[4];//initialize here
        servos = new RegServo[12];//initialize here
        CRservos = new ContRotServo[12];
        CRservos[0] = new ContRotServo(hardwareMap.get(CRServo.class,"turretServo1"));
        CRservos[1] = new ContRotServo(hardwareMap.get(CRServo.class,"turretServo2"));
        hub2Motors[0] = new Motor(hardwareMap.get(DcMotorEx.class,"shooterMotor1"));
        hub2Motors[1] = new Motor(hardwareMap.get(DcMotorEx.class,"shooterMotor2"));
        hub2Motors[2] = new Motor(hardwareMap.get(DcMotorEx.class,"intakeMotor1"));
        hub2Motors[3] = new Motor(hardwareMap.get(DcMotorEx.class,"intakeMotor2"));
        servos[0] = new RegServo(hardwareMap.get(Servo.class,"shootAngleController"));
        servos[1] = new RegServo(hardwareMap.get(Servo.class,"intakeDropperGuard"));
        servos[2] = new RegServo(hardwareMap.get(Servo.class,"magServo"));
        servos[4] = new RegServo(hardwareMap.get(Servo.class,"wobblerClaw"));
        servos[5] = new RegServo(hardwareMap.get(Servo.class,"wobblerArm"));
        servos[6] = new RegServo(hardwareMap.get(Servo.class,"ringPusher"));
        servos[7] = new RegServo(hardwareMap.get(Servo.class, "magRotationServo"));
        shooter = new Shooter(hub2Motors[0],hub2Motors[1],servos[0],this);
        turret = new Turret(new ContRotServo[]{CRservos[0],CRservos[1]}, servos[7], hub2Motors[0], this);
        intake = new Intake(hub2Motors[2],hub2Motors[3],servos[1]);
        mag = new Mag(servos[2],servos[6]);
        wobbler = new WobblerArm(servos[5],servos[4]);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(){
        loops++;
        boolean hub1ReadNeeded = true;
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
        drive.update();
        currentPose = drive.getPoseEstimate();
        poseStorage = currentPose;


        boolean hub2ReadNeeded = false;
        for(Motor motor: hub2Motors){
            if(motor != null && motor.readRequested)
                hub2ReadNeeded = true;
        }
        if(hub2ReadNeeded) {
            allHubs.get(1).clearBulkCache();
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
        if(shooter.updatePID) {
            shooter.updateShooterPIDF(deltaTimeHub2 / 1000);
        }
        if(turret.updatePID){
            turret.updateTurretPID();
        }
        for(Motor motor: hub2Motors){
            if(motor!=null&&motor.setTargetPosRequested){
                motor.motor.setTargetPosition(motor.targetPosition);
                motor.setTargetPosRequested = false;
            }
        }
        for(Motor motor: hub2Motors){
            if(motor!=null&&motor.writePowerRequested){
                motor.motor.setPower(motor.power);
                motor.writePowerRequested = false;
            }
            if(motor!=null&&motor.writeVelocityRequested){
                motor.motor.setVelocity(motor.velocity);
                motor.writeVelocityRequested = false;
            }
        }
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
        return currentPose.getHeading();
    }
    public static HardwareMecanum getInstance(){
        return hw;
    }
}

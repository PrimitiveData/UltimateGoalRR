package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
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
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Ramsete.Pose;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.ThreeWheelTrackingLocalizerAnalogGyro;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Intake;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Shooter;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Turret;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.WobblerArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

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
    public ElapsedTime timer;
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
    public static volatile Pose2d poseStorage = new Pose2d(-63,-47,Math.PI);
    public static volatile double PoseStorageX;
    public static volatile double PoseStorageY;
    public static volatile double PoseStorageHeading;
    public static double cumulativeAngleStorage=Math.PI;
    public double cumulativeAngle = Math.PI;
    public double prevAngle;
    public boolean updateDrivePID = false;
    public Pose2d targetPose = new Pose2d(-63,-47,Math.PI);
    public TelemetryPacket packet;
    public static boolean autoRan;
    public HardwareMecanum(HardwareMap hardwareMap, Telemetry telemetry, boolean sanfordGyroLocalizer){
        this.hardwareMap = hardwareMap;
        hw = this;
        HardwareMecanum.telemetry = telemetry;
        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        if(poseStorage == null){
            currentPose = new Pose2d(-63,-47, Math.PI);
        }else{
            currentPose = poseStorage;
        }
//        if (PoseStorageX == 0.0)
//            currentPose = new Pose2d(-63, -47, Math.PI);
//        else
//            currentPose = new Pose2d(PoseStorage.PoseStorageX, PoseStorage.PoseStorageY, PoseStorage.PoseStorageHeading);

//        if (autoRan)
//            currentPose = new Pose2d(PoseStorageX, PoseStorageY, PoseStorageHeading);
//        else
//            currentPose = new Pose2d(-63, -47, Math.PI);
//        autoRan = false;
        /*
        if(poseStorage == null){
            currentPose = new Pose2d(-63, -47, Math.PI);
        }else{
            currentPose = poseStorage;
        }
         */
        hub1Motors = new Motor[4];//initialize here
        time = new ElapsedTime();
        timer = new ElapsedTime();
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
        if(sanfordGyroLocalizer) {
            drive = new SampleMecanumDrive(hardwareMap, true);
        }
        else{
            drive = new SampleMecanumDrive(hardwareMap);
        }

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        packet = new TelemetryPacket();
    }
    public HardwareMecanum(HardwareMap hardwareMap, Telemetry telemetry){
        this(hardwareMap,telemetry, false);
    }

    public void loop(){
//        System.out.println(timer.milliseconds());
        try {
            System.out.println("Pose Storage:" + poseStorage.getX() + poseStorage.getY() + poseStorage.getHeading());
        }
        catch (Exception e){
            System.out.println("exception caught");
        }
        timer.reset();
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
                motor.currentVelocity = motor.motor.getVelocity();
            }
        }
        if(firstLoop){
            startTimeHub1 = time.milliseconds();
            prevTimeHub1 = startTimeHub1;
        }
        double currentTimeHub1 = time.milliseconds();
        deltaTimeHub1 = currentTimeHub1-prevTimeHub1;
        prevTimeHub1 = currentTimeHub1;
        if(shooter.updatePID) {
            shooter.updateShooterPIDF();
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
        telemetry.update();
        
        if(updateDrivePID) {
            drive.updateDrivetrainPID(drive.getPoseEstimate(), targetPose);
            drive.updateDrivetrainHeadingPID(drive.getPoseEstimate(), targetPose);
        }
        for(int i = 0; i <hub2Motors.length;i++){
            Motor motor = hub2Motors[i];
            if(motor != null && motor.readRequested){
                motor.currentPosition = motor.motor.getCurrentPosition();
                motor.currentVelocity = motor.motor.getVelocity();
            }
        }
        if(firstLoop){
            startTimeHub2 = time.milliseconds();
            prevTimeHub2 = startTimeHub2;
            prevAngle = currentPose.getHeading();
            firstLoop = false;
        }
        double currentTimeHub2 = time.milliseconds();
        deltaTimeHub2 = currentTimeHub2-prevTimeHub2;
        prevTimeHub2 = currentTimeHub2;

        currentPose = drive.getPoseEstimate();

        cumulativeAngle += MathFunctions.keepAngleWithin180Degrees(currentPose.getHeading() - prevAngle);
        cumulativeAngleStorage = cumulativeAngle;
        packet.put("prevAngle",MathFunctions.keepAngleWithin180Degrees(prevAngle));
        prevAngle = currentPose.getHeading();
        packet.put("cumulativeAngle",MathFunctions.keepAngleWithin180Degrees(cumulativeAngle));
        packet.put("RRheading",MathFunctions.keepAngleWithin180Degrees(currentPose.getHeading()));
        Canvas fieldOverlay = packet.fieldOverlay();
        //FtcDashboard.getInstance().sendTelemetryPacket(packet);

//        PoseStorageX = currentPose.getX();
//        PoseStorageY = currentPose.getY();
//        PoseStorageHeading = currentPose.getHeading();
//        PoseStorage.PoseStorageX = PoseStorageX;
//        PoseStorage.PoseStorageY = PoseStorageY;
//        PoseStorage.PoseStorageHeading = PoseStorageHeading;
        poseStorage = currentPose;

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

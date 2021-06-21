package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.TimedAction;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;
import org.firstinspires.ftc.teamcode.hardware.PID.ShooterPID;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Config
@Autonomous(name = "RedMTINoStack", group = "Autonomous")
public class RMTINoStack extends LinearOpMode {
    public static double velo = 0;
    public static boolean updateTurret = true;
    TimedAction flicker;

    //state machine
    public enum State{
        STARTED, //shooter off, turns on turret tracking
        ARMDOWN, //moves wobble arm to drop wobble
        OPENCLAW, //opens wobble claw to drop wobble
        ARMUP, //closes claw, raises wobble to rest pos
        SHOOTERON, //turns shooter on
        SHOOTERSHOOT, //moves flicker to shoot rings
        SHOOTEROFF, //turns shooter off
        IDLE //nothing happens here (end of auto state)
    }
    public State state = State.IDLE;

    Pose2d startPose = new Pose2d(-63, -47, Math.PI); //starting pose
    Vector2d shooterVector = new Vector2d(-8,-55); //shooting vector

    @Override
    public void runOpMode() {
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry, false);
        hardware.shooter.shooterVeloPID = new ShooterPID(0,0,0,0.004893309156,3.238478883,0,Double.POSITIVE_INFINITY,hardware.time,"/sdcard/FIRST/shooterFFdata.txt");
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(hardware, this);
        hardware.turret.turretMotor.readRequested = true;
        hardware.shooter.updatePID = true;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //servo init
        hardware.wobbler.goToWobbleStartingPos();
        hardware.wobbler.gripWobble();
        hardware.mag.setRingPusherResting();
        hardware.mag.dropRings();
        hardware.intake.holdIntakeUp();
        hardware.loop();


        //flicker timing
        Servo flickServo = hardwareMap.get(Servo.class, "ringPusher");
        flicker = new TimedAction(
                ()->flickServo.setPosition(0.7),
                ()->flickServo.setPosition(0.92),
                200,
                true
        );

        //paths
        //zero stack path
        TrajectorySequence zeroStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(0.1, () -> state = State.STARTED)
                .addTemporalMarker(1, ()->hardware.intake.dropIntake())
                .splineTo(new Vector2d(-1,-59), 0)
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.ARMDOWN)
                .UNSTABLE_addTemporalMarkerOffset(1.25, ()-> state = State.OPENCLAW)
                .UNSTABLE_addTemporalMarkerOffset(2.49, ()-> state = State.ARMUP)
                .waitSeconds(2.5)
                .lineTo(shooterVector)
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(3.25, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(4.9, ()-> state = State.SHOOTEROFF)
                .waitSeconds(5)
                .forward(40)
                .waitSeconds(13)
                .splineToLinearHeading(new Pose2d(14, -36, Math.toRadians(1)), 0)
                .addTemporalMarker(0.9, 0.1, ()->state = State.IDLE)
                .build();
        //one stack path
        TrajectorySequence oneStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(0.1, () -> state = State.STARTED)
                .addTemporalMarker(1, ()->hardware.intake.dropIntake())
                .splineToConstantHeading(shooterVector, 0) //shooting
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(3.25, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(4.9, ()-> state = State.SHOOTEROFF)
                .waitSeconds(5)
                .splineToConstantHeading(new Vector2d(24, -37), 0) //wobble
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.ARMDOWN)
                .UNSTABLE_addTemporalMarkerOffset(1.25, ()-> state = State.OPENCLAW)
                .UNSTABLE_addTemporalMarkerOffset(2.49, ()-> state = State.ARMUP)
                .waitSeconds(2.5)
                .lineToLinearHeading(new Pose2d(10,-58, Math.toRadians(90))) //park
                .addTemporalMarker(0.9, 0.1, ()->state = State.IDLE)
                .build();
        //four stack path
        TrajectorySequence fourStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(0.1, () -> state = State.STARTED)
                .addTemporalMarker(1, ()->hardware.intake.dropIntake())
                .splineTo(shooterVector, 0) //shooting
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(3.25, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(4.9, ()-> state = State.SHOOTEROFF)
                .waitSeconds(5)
                .splineTo(new Vector2d(48, -59), 0) //wobble
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.ARMDOWN)
                .UNSTABLE_addTemporalMarkerOffset(1.25, ()-> state = State.OPENCLAW)
                .UNSTABLE_addTemporalMarkerOffset(2.49, ()-> state = State.ARMUP)
                .waitSeconds(2.5)
                .lineToSplineHeading(new Pose2d(10,-56, Math.toRadians(90))) //park
                .addTemporalMarker(0.9, 0.1, ()->state = State.IDLE)
                .build();

        //camera control
        UGContourRingDetector detector;
        UGContourRingPipeline.Height height;
        detector = new UGContourRingDetector(hardwareMap, "Webcam 1", telemetry, true);
        detector.init();
        height = detector.getHeight();
        while(!opModeIsActive()){
            height = detector.getHeight();
        }

        waitForStart();
        drive.setPoseEstimate(startPose);

        //chooses path based on vision data
        switch (height) {
            case ZERO:
                drive.followTrajectorySequenceAsync(zeroStack);
                break;
            case ONE:
                drive.followTrajectorySequenceAsync(oneStack);
                break;
            case FOUR:
                drive.followTrajectorySequenceAsync(fourStack);
                break;
        }

        while(!isStopRequested()){
            //hardware and drive updating
            drive.update();
            hardware.loop();
            //turret autoaim update
            hardware.turret.update(velo, drive.getPoseEstimate(), true, updateTurret);

            hardware.shooter.updateShooterPIDF();

            telemetry.addData("Requested shooter velo: ", velo);
            telemetry.addData("State", state);
            telemetry.update();

            //states for auto
            switch (state) {
                case STARTED:
                    hardware.turret.updatePID = true;
                    velo = 0;
                    break;
                case ARMDOWN:
                    hardware.wobbler.goToWobblerDropPosition();
                    break;
                case OPENCLAW:
                    hardware.wobbler.releaseWobble();
                    break;
                case ARMUP:
                    hardware.wobbler.goToClawRestingPos();
                    hardware.wobbler.goToArmRestingPos();
                    break;
                case SHOOTERON:
                    velo = 1225;
                    break;
                case SHOOTERSHOOT:
                    shoot();
                    break;
                case SHOOTEROFF:
                    velo = 0;
                    updateTurret = false;
                    break;
                case IDLE:
                    break;
            }
        }
    }
    //method to shoot
    public void shoot(){
        if(!flicker.running())
            flicker.reset();
        flicker.run();
    }
}

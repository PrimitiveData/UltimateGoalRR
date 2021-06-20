package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Ramsete.Pose;
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

    Pose2d startPose = new Pose2d(-63.25, -54, Math.PI); //starting pose
    Vector2d shooterVector = new Vector2d(-8,-59); //shooting vector

    @Override
    public void runOpMode() {
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry, false);
        hardware.shooter.shooterVeloPID = new ShooterPID(0,0,0,0.004893309156,3.238478883,0,Double.POSITIVE_INFINITY,hardware.time,"/sdcard/FIRST/shooterFFdata.txt");
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(hardware, this);
        hardware.turret.turretMotor.readRequested = true;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        hardware.wobbler.goToWobbleStartingPos();
        hardware.wobbler.gripWobble();
        hardware.mag.setRingPusherResting();
        hardware.mag.dropRings();
        hardware.intake.holdIntakeUp();


        //flicker timing
        flicker = new TimedAction(
                ()->hardware.mag.ringPusher.setPosition(0.46),
                ()->hardware.mag.ringPusher.setPosition(0.69),
                200,
                true
        );

        //paths
        //zero stack path
        TrajectorySequence zeroStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addDisplacementMarker(()->state = State.STARTED)
                .splineTo(shooterVector, 0) //shooting
                .addTemporalMarker(1, ()-> hardware.intake.dropIntake()) //time after starting path to drop intake
                .addDisplacementMarker(()->state = State.ARMDOWN)
                .waitSeconds(1)
                .addDisplacementMarker(()->state = State.OPENCLAW)
                .waitSeconds(0.5)
                .addDisplacementMarker(()->state = State.ARMUP)
                .waitSeconds(1.5)
                .addDisplacementMarker(()->state = State.SHOOTERON)
                .waitSeconds(1)
                .addDisplacementMarker(()->state = State.SHOOTERSHOOT)
                .waitSeconds(3)
                .addDisplacementMarker(()->state = State.SHOOTEROFF)
                .forward(50)
                .waitSeconds(15)
                .splineToLinearHeading(new Pose2d(14, -36,
                        new Vector2d(14, -36).angleBetween(new Vector2d(72, -22)) //park
                ), 0)
                .addDisplacementMarker(()->state = State.IDLE)
                .build();
        //one stack path
        TrajectorySequence oneStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addDisplacementMarker(()->state = State.STARTED)
                .splineToConstantHeading(shooterVector, 0) //shooting
                .addTemporalMarker(1, ()-> hardware.intake.dropIntake()) //time after starting path to drop intake
                .addDisplacementMarker(()->state = State.SHOOTERON)
                .waitSeconds(1)
                .addDisplacementMarker(()->state = State.SHOOTERSHOOT)
                .waitSeconds(3)
                .addDisplacementMarker(()->state = State.SHOOTEROFF)
                .splineToConstantHeading(new Vector2d(24, -37), 0) //wobble
                .addDisplacementMarker(()->state = State.ARMDOWN)
                .waitSeconds(1)
                .addDisplacementMarker(()->state = State.OPENCLAW)
                .waitSeconds(0.5)
                .addDisplacementMarker(()->state = State.ARMUP)
                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(10,-58, Math.toRadians(90))) //park
                .addDisplacementMarker(()->state = State.IDLE)
                .build();
        //four stack path
        TrajectorySequence fourStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addDisplacementMarker(()->state = State.STARTED)
                .splineTo(shooterVector, 0) //shooting
                .addTemporalMarker(1, ()-> hardware.intake.dropIntake()) //time after starting path to drop intake
                .addDisplacementMarker(()->state = State.SHOOTERON)
                .waitSeconds(1)
                .addDisplacementMarker(()->state = State.SHOOTERSHOOT)
                .waitSeconds(3)
                .addDisplacementMarker(()->state = State.SHOOTEROFF)
                .splineTo(new Vector2d(48, -59), 0) //wobble
                .addDisplacementMarker(()->state = State.ARMDOWN)
                .waitSeconds(1)
                .addDisplacementMarker(()->state = State.OPENCLAW)
                .waitSeconds(0.5)
                .addDisplacementMarker(()->state = State.ARMUP)
                .waitSeconds(1.5)
                .lineToSplineHeading(new Pose2d(10,-56, Math.toRadians(90))) //park
                .addDisplacementMarker(()->state = State.IDLE)
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
            drive.update();
            hardware.turret.update(velo, drive.getPoseEstimate(), true);
            hardware.loop();

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

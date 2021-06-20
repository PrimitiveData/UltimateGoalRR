package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.TimedAction;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;
import org.firstinspires.ftc.teamcode.hardware.PID.ShooterPID;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedMTINoStack", group = "Autonomous")
public class RMTINoStack extends LinearOpMode {
    private double velo = 0;
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

    @Override
    public void runOpMode() {
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry, false);
        hardware.shooter.shooterVeloPID = new ShooterPID(0,0,0,0.004893309156,3.238478883,0,Double.POSITIVE_INFINITY,hardware.time,"/sdcard/FIRST/shooterFFdata.txt");
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(hardware, this);
        hardware.turret.turretMotor.readRequested = true;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //flicker timing
        flicker = new TimedAction(
                ()->hardware.mag.ringPusher.setPosition(0.46),
                ()->hardware.mag.ringPusher.setPosition(0.69),
                200,
                true
        );

        //paths
        TrajectorySequence zeroStack = drive.trajectorySequenceBuilder(new Pose2d(-63.25, -54, Math.PI))
                .setReversed(true)
                .addDisplacementMarker(()->state = State.STARTED)
                .splineTo(new Vector2d(-8, -59), 0)
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
                        new Vector2d(14, -36).angleBetween(new Vector2d(72, -22))
                ), 0)
                .addDisplacementMarker(()->state = State.IDLE)
                .build();
        TrajectorySequence oneStack = drive.trajectorySequenceBuilder(new Pose2d(-63.25, -54, Math.PI))
                .setReversed(true)
                .addDisplacementMarker(()->state = State.STARTED)
                .splineToConstantHeading(new Vector2d(-6, -58), 0)
                .addDisplacementMarker(()->state = State.SHOOTERON)
                .waitSeconds(1)
                .addDisplacementMarker(()->state = State.SHOOTERSHOOT)
                .waitSeconds(3)
                .addDisplacementMarker(()->state = State.SHOOTEROFF)
                .splineToConstantHeading(new Vector2d(24, -37), 0)
                .addDisplacementMarker(()->state = State.ARMDOWN)
                .waitSeconds(1)
                .addDisplacementMarker(()->state = State.OPENCLAW)
                .waitSeconds(0.5)
                .addDisplacementMarker(()->state = State.ARMUP)
                .waitSeconds(1.5)
                .splineToConstantHeading(new Vector2d(10, -44), 0)
                .splineTo(new Vector2d(10,-58), Math.toRadians(-90))
                .addDisplacementMarker(()->state = State.IDLE)
                .build();
        TrajectorySequence fourStack = drive.trajectorySequenceBuilder(new Pose2d(-63.25, -54, Math.PI))
                .setReversed(true)
                .addDisplacementMarker(()->state = State.STARTED)
                .splineTo(new Vector2d(-6, -59), 0)
                .addDisplacementMarker(()->state = State.SHOOTERON)
                .waitSeconds(1)
                .addDisplacementMarker(()->state = State.SHOOTERSHOOT)
                .waitSeconds(3)
                .addDisplacementMarker(()->state = State.SHOOTEROFF)
                .splineTo(new Vector2d(48, -59), 0)
                .addDisplacementMarker(()->state = State.ARMDOWN)
                .waitSeconds(1)
                .addDisplacementMarker(()->state = State.OPENCLAW)
                .waitSeconds(0.5)
                .addDisplacementMarker(()->state = State.ARMUP)
                .waitSeconds(1.5)
                .lineToSplineHeading(new Pose2d(10,-56, Math.toRadians(90)))
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

        while(!isStopRequested()){
            drive.update();
            hardware.turret.update(velo, drive.getPoseEstimate(), true);

            switch (height) {
                case ZERO:
                    drive.followTrajectorySequenceAsync(zeroStack);
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
                    break;
                case ONE:
                    drive.followTrajectorySequenceAsync(oneStack);
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
                    break;
                case FOUR:
                    drive.followTrajectorySequenceAsync(fourStack);
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

package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Ramsete.Pose;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.TimedAction;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;
import org.firstinspires.ftc.teamcode.hardware.PID.ShooterPID;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Config
@Autonomous(name = "BlueMTIStack", group = "Autonomous")
public class BMTI extends LinearOpMode {
    public static double velo = 0;
    public static boolean updateTurret = false;
    TimedAction flicker;

    //state machine
    public enum State{
        STARTED, //shooter off
        TURRETON, //turns turret tracking on
        ARMDOWN, //moves wobble arm to drop wobble
        ARMDOWN2, //moves wobble arm to final drop pos
        OPENCLAW, //opens wobble claw to drop wobble
        ARMUP, //closes claw, raises wobble to rest pos
        SHOOTERON, //turns shooter on
        SHOOTERSHOOT, //moves flicker to shoot rings
        SHOOTEROFF, //turns shooter off
        INTAKEON, //turns intake on
        INTAKEOFF, //turns intake off
        RESET, //reset turret + offsets
        STOP //stop (end of auto state)
    }
    public State state = State.STARTED;

    Pose2d startPose = new Pose2d(-63, 47, Math.PI); //starting pose

    @Override
    public void runOpMode() {
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry, false);
        hardware.shooter.shooterVeloPID = new ShooterPID(0,0,0,0.004893309156,3.238478883,0,Double.POSITIVE_INFINITY,hardware.time,"/sdcard/FIRST/shooterFFdata.txt");
        hardware.turret.turretMotor.readRequested = true;
        hardware.shooter.updatePID = true;

        hardware.turret.turretMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.turret.turretMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.turret.turretMotor.motor.setDirection(DcMotorEx.Direction.REVERSE);

        SampleMecanumDrive drive = hardware.drive;
        HardwareMecanum.poseStorage = startPose;
        drive.setPoseEstimate(startPose);
        hardware.autoRan = true;

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
                .waitSeconds(17)
                .splineTo(new Vector2d(-11,60), 0) //wobble and shooting
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.ARMDOWN)
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> state = State.ARMDOWN2)
                .UNSTABLE_addTemporalMarkerOffset(1.25, ()-> state = State.OPENCLAW)
                .UNSTABLE_addTemporalMarkerOffset(2.49, ()-> state = State.ARMUP)
                .waitSeconds(2.5) //total time for wobble sequence
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> hardware.intake.dropIntake())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> state = State.TURRETON)
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()-> hardware.intake.raiseBumper())
                .UNSTABLE_addTemporalMarkerOffset(3.25, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(4.9, ()-> state = State.SHOOTEROFF)
                .UNSTABLE_addTemporalMarkerOffset(4.95, ()-> state = State.RESET)
                .waitSeconds(5) //total time for shooting sequence
                .splineToLinearHeading(new Pose2d(14, 36, Math.toRadians(-1)), 0) //park
                .addTemporalMarker(29.9, ()->state = State.STOP)
                .build();
        //one stack path
        TrajectorySequence oneStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .waitSeconds(9)
                .splineToSplineHeading(new Pose2d(-11,58, Math.toRadians(160)), 0) //shooting
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> hardware.intake.dropIntake())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> state = State.TURRETON)
                .UNSTABLE_addTemporalMarkerOffset(2.0, ()-> hardware.intake.raiseBumper())
                .UNSTABLE_addTemporalMarkerOffset(3.25, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(4.9, ()-> state = State.SHOOTEROFF)
                .UNSTABLE_addTemporalMarkerOffset(4.95, ()-> state = State.RESET)
                .UNSTABLE_addTemporalMarkerOffset(4.99, ()-> state = State.INTAKEON)
                .waitSeconds(5)
                .lineToLinearHeading(new Pose2d(-18, 38, Math.toRadians(240))) //stack
                .splineTo(new Vector2d(10, 48), 0) //wobble
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.INTAKEOFF)
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> state = State.ARMDOWN)
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> state = State.ARMDOWN2)
                .UNSTABLE_addTemporalMarkerOffset(1.25, ()-> state = State.OPENCLAW)
                .UNSTABLE_addTemporalMarkerOffset(2.49, ()-> state = State.ARMUP)
                .waitSeconds(2.5)
                .lineToLinearHeading(new Pose2d(0,58, Math.toRadians(-90))) //shoot
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> state = State.TURRETON)
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(3.0, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(3.8, ()-> state = State.SHOOTEROFF)
                .UNSTABLE_addTemporalMarkerOffset(3.95, ()-> state = State.RESET)
                .waitSeconds(4)
                .lineTo(new Vector2d(10, 58))
                .addDisplacementMarker(()-> state = State.STOP)
                .build();
        //four stack path
        TrajectorySequence fourStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-11,58, Math.toRadians(160)), 0) //shooting
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> hardware.intake.dropIntake())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> state = State.TURRETON)
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()-> hardware.intake.raiseBumper())
                .UNSTABLE_addTemporalMarkerOffset(2.0, ()-> hardware.intake.dropIntake())
                .UNSTABLE_addTemporalMarkerOffset(2.0, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(3.4, ()-> state = State.SHOOTEROFF)
                .UNSTABLE_addTemporalMarkerOffset(3.45, ()-> state = State.RESET)
                .waitSeconds(3.5)
                .splineToSplineHeading(new Pose2d(34, 56, Math.toRadians(225)), 0) //wobble
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.ARMDOWN)
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> state = State.ARMDOWN2)
                .UNSTABLE_addTemporalMarkerOffset(1.25, ()-> state = State.OPENCLAW)
                .UNSTABLE_addTemporalMarkerOffset(1.7, ()-> state = State.ARMUP)
                .waitSeconds(1.75)
                .setReversed(false)
                .splineTo(new Vector2d(10, 40), Math.PI) //spline to stack
                .splineTo(new Vector2d(-12, 34), Math.PI) //spline to stack
                .addDisplacementMarker(()-> state = State.INTAKEON)
                .forward(16,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()-> state = State.INTAKEOFF)
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> state = State.TURRETON)
                .UNSTABLE_addTemporalMarkerOffset(3, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(4.4, ()-> state = State.SHOOTEROFF)
                .UNSTABLE_addTemporalMarkerOffset(4.49, ()-> state = State.INTAKEON)
                .waitSeconds(4.5) //total time for shooting sequence
                .forward(12,
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()-> state = State.INTAKEOFF)
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(3, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(4.4, ()-> state = State.SHOOTEROFF)
                .UNSTABLE_addTemporalMarkerOffset(4.49, ()-> state = State.RESET)
                .waitSeconds(4.5) //total time for shooting sequence
                .lineToLinearHeading(new Pose2d(10, 58, -Math.PI/2)) //park
                .addTemporalMarker(29.9, ()->state = State.STOP)
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
            hardware.loop();
            //turret autoaim update
            hardware.turret.update(velo, drive.getPoseEstimate(), false, updateTurret);

            hardware.shooter.updateShooterPIDF();

            telemetry.addData("Requested shooter velo: ", velo);
            telemetry.addData("State", state);
            telemetry.update();

            //states for auto
            switch (state) {
                case STARTED:
                    hardware.turret.maxPositive = Math.toRadians(0);
                    hardware.turret.maxNegative = Math.toRadians(-200);
                    hardware.turret.updatePID = true;
                    velo = 0;
                    break;
                case ARMDOWN:
                    hardware.wobbler.goToWobblerDropPosition();
                    break;
                case ARMDOWN2:
                    hardware.wobbler.moveArmToGrabPos();
                    break;
                case OPENCLAW:
                    hardware.wobbler.releaseWobble();
                    break;
                case ARMUP:
                    hardware.turret.maxPositive = Math.toRadians(315);
                    hardware.turret.maxNegative = Math.toRadians(-225);
                    hardware.wobbler.goToClawRestingPos();
                    hardware.wobbler.goToArmRestingPos();
                    break;
                case SHOOTERON:
                    hardware.mag.dropRings();
                    velo = 1350;
                    break;
                case SHOOTERSHOOT:
                    shoot();
                    break;
                case SHOOTEROFF:
                    velo = 0;
                    hardware.mag.setRingPusherResting();
                    hardware.mag.collectRings();
                    break;
                case INTAKEON:
                    hardware.intake.turnIntake(1);
                    break;
                case INTAKEOFF:
                    hardware.intake.turnIntake(0);
                    break;
                case TURRETON:
                    updateTurret = true;
                    break;
                case RESET:
                    hardware.turret.setLocalTurretAngle(0);
                    updateTurret = false;
                    break;
                case STOP:
                    hardware.turret.updatePID = false;
                    hardware.turret.turretAngleOffsetAdjustmentConstant = 0;
                    hardware.shooter.rampAngleAdjustmentConstant = 0;
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

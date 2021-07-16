package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.TimedAction;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.PID.ShooterPID;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

@Config
@Autonomous(name = "RedMTIStack", group = "Autonomous")
public class RMTI extends LinearOpMode {
    public static double velo = 0;
    public static boolean updateTurret = true;
    TimedAction flicker;

    //state machine
    public enum State{
        STARTED, //shooter off, turns on turret tracking
        ARMDOWN, //moves wobble arm to drop wobble
        ARMDOWN2, //moves wobble arm to final drop pos
        OPENCLAW, //opens wobble claw to drop wobble
        ARMUP, //closes claw, raises wobble to rest pos
        SHOOTERON, //turns shooter on
        SHOOTERSHOOT, //moves flicker to shoot rings
        SHOOTEROFF, //turns shooter off
        INTAKEON, //turns intake on
        INTAKEOFF, //turns intake off
        RESET, //nothing happens here (end of auto state)
        STOP //stop reading odo
    }
    public State state = State.STARTED;

    Pose2d startPose = new Pose2d(-63, -47, Math.PI); //starting pose

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
                200 ,
                true
        );

        //paths
        //zero stack path
        TrajectorySequence zeroStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(0.1, () -> state = State.STARTED)
                .addTemporalMarker(18, ()->hardware.intake.dropIntake()) //time to wait before dropping intake
                .addTemporalMarker(20, ()->hardware.intake.raiseBumper()) //bring bumper back up
                .waitSeconds(11)
                .splineTo(new Vector2d(-11,-58), 0) //wobble
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.ARMDOWN)
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> state = State.ARMDOWN2)
                .UNSTABLE_addTemporalMarkerOffset(1.25, ()-> state = State.OPENCLAW)
                .UNSTABLE_addTemporalMarkerOffset(2.49, ()-> state = State.ARMUP)
                .waitSeconds(2.5) //total time for wobble sequence
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(3.25, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(4.9, ()-> state = State.SHOOTEROFF)
                .waitSeconds(5) //total time for shooting sequence
                .lineTo(new Vector2d(-14, -42)) //park
                .addDisplacementMarker(()->state = State.RESET)
                .lineToLinearHeading(new Pose2d(10, -42, 0)) //park
                .addTemporalMarker(29.9, ()->state = State.STOP)
                .build();
        //one stack path
        TrajectorySequence oneStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(0.1, () -> state = State.STARTED)
                .waitSeconds(4)
                .lineToConstantHeading(new Vector2d(-6, -58)) //shooting
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(3.25, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(4.85, ()-> state = State.SHOOTEROFF)
                .waitSeconds(5) //total time for shooting sequence
                .splineToConstantHeading(new Vector2d(10, -36), 0) //wobble
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> hardware.intake.dropIntake())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> hardware.intake.raiseBumper())
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()-> state = State.ARMDOWN)
                .UNSTABLE_addTemporalMarkerOffset(1.75, ()-> state = State.OPENCLAW)
                .UNSTABLE_addTemporalMarkerOffset(2.9, ()-> state = State.ARMUP)
                .UNSTABLE_addTemporalMarkerOffset(2.99, ()-> state = State.INTAKEON)
                .waitSeconds(3) //total time for wobble sequence
                .lineToConstantHeading(new Vector2d(-20, -39)) //stack pickup
                .UNSTABLE_addTemporalMarkerOffset(2, () -> state = State.INTAKEOFF)
                .UNSTABLE_addTemporalMarkerOffset(2.1, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(5.25, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(6.85, ()-> state = State.SHOOTEROFF)
                .UNSTABLE_addTemporalMarkerOffset(6.9, ()-> state = State.RESET)
                .waitSeconds(7) //total time for shooting sequence
                .lineToLinearHeading(new Pose2d(10, -58, Math.PI/2))
                .addTemporalMarker(29.9, ()->state = State.STOP)
                .build();
        //four stack path
        TrajectorySequence fourStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(0.1, () -> state = State.STARTED)
                .splineTo(new Vector2d(-8,-58), 0) //shooting
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> hardware.intake.dropIntake())
                .UNSTABLE_addTemporalMarkerOffset(0.15, ()-> state = State.SHOOTERON)
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> hardware.intake.raiseBumper())
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()-> hardware.intake.dropIntake())
                .UNSTABLE_addTemporalMarkerOffset(2.15, ()-> state = State.SHOOTERSHOOT)
                .UNSTABLE_addTemporalMarkerOffset(3.45, ()-> state = State.SHOOTEROFF)
                .waitSeconds(3.5) //total time for shooting sequence
                .lineToLinearHeading(new Pose2d(-15, -47, Math.toRadians(135)))
                .addDisplacementMarker(()-> state = State.INTAKEON)
                .forward(16,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()-> state = State.INTAKEOFF)
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()-> state = State.SHOOTERON)
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
                .splineToLinearHeading(new Pose2d(0, -59, Math.PI), 0) //wobble
                .splineToLinearHeading(new Pose2d(34, -59, Math.PI), 0) //wobble
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> state = State.ARMDOWN)
                .UNSTABLE_addTemporalMarkerOffset(1.25, ()-> state = State.OPENCLAW)
                .UNSTABLE_addTemporalMarkerOffset(2.49, ()-> state = State.ARMUP)
                .waitSeconds(2.5) //total time for wobble sequence
                .lineToSplineHeading(new Pose2d(10,-56, Math.toRadians(90))) //park
                .addTemporalMarker(29.9, ()->state = State.STOP)
                .build();

        /*
            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
         */

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
                hardware.shooter.rampAngleAdjustmentConstant = -0.001;
                drive.followTrajectorySequenceAsync(fourStack);
                break;
        }

        while(opModeIsActive()){
            //hardware and drive updating
            hardware.loop();
            //turret autoaim update
            hardware.turret.update(velo, drive.getPoseEstimate(), true, updateTurret);

            hardware.shooter.updateShooterPIDF();

            TrajectorySequenceRunner.turretHeading = Math.toDegrees(hardware.turret.localTurretAngleRadians());

            double[] turretPosition = MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),-4.22,hardware.getAngle());
            double distanceToGoal = Math.hypot(turretPosition[1]- FieldConstants.highGoalPosition[1],turretPosition[0] - FieldConstants.highGoalPosition[0]);
            telemetry.addData("Requested shooter velo: ", velo);
            telemetry.addData("State", state);
            telemetry.addData("Turret heading:", Math.toDegrees(hardware.turret.localTurretAngleRadians()));
            telemetry.addData("Angle to goal:", Math.toDegrees(Math.atan2(FieldConstants.highGoalPosition[1]-turretPosition[1], FieldConstants.highGoalPosition[0]-turretPosition[0]) + hardware.turret.getTurretOffset(distanceToGoal)));
            telemetry.addData("Distance to goal:", distanceToGoal);
            telemetry.addData("Record pose:", hardware.drive.recordPose);
            telemetry.update();

            //states for auto
            switch (state) {
                case STARTED:
                    hardware.turret.updatePID = true;
                    velo = 0;
                    hardware.turret.maxPositive = Math.toRadians(0);
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
                    hardware.wobbler.goToClawRestingPos();
                    hardware.wobbler.goToArmRestingPos();
                    hardware.turret.maxPositive = Math.toRadians(315);
                    break;
                case SHOOTERON:
//                    hardware.turret.turretAngleOffsetAdjustmentConstant = Math.toRadians(1);
//                    hardware.shooter.rampAngleAdjustmentConstant = -0.025;
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
                case RESET:
                    hardware.turret.setLocalTurretAngle(0);
                    updateTurret = false;
                    break;
                case STOP:
                    hardware.turret.updatePID = false;
//                    hardware.drive.recordPose = false;
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

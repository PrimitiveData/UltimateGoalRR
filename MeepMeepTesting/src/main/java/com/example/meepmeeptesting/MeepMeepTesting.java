//package com.example.meepmeeptesting;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
//import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
//import com.noahbres.meepmeep.roadrunner.Constraints;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
//
//public class MeepMeepTesting {
//    public static void main(String[] args) {
//        Vector2d shooterVector = new Vector2d(-8,-58); //shooting vector
//
//        MeepMeep mm = new MeepMeep(650)
//                // Set field image
//                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
//                // Set theme
//                .setTheme(new ColorSchemeRedDark())
//                // Background opacity from 0-1
//                .setBackgroundAlpha(1.0f)
//                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(62, 62, Math.toRadians(210), Math.toRadians(210), 15.74)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-63.25, -15, 0))
//                                .addDisplacementMarker(() -> {
//                                })
//                                .lineTo(new Vector2d(-6, -15))
//                                .waitSeconds(3)
//                                .splineTo(new Vector2d(24, -25), -Math.PI / 2)
//                                .lineTo(new Vector2d(24, -52))
//                                .splineTo(new Vector2d(32, -57), 0)
//                                .splineTo(new Vector2d(40, -52), Math.PI / 2)
//                                .lineTo(new Vector2d(40, -25))
//                                .splineTo(new Vector2d(55, -15), 0)
//                                .turn(Math.PI / 2)
//                                .lineTo(new Vector2d(58, -41))
//                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
//                                })
//                                .waitSeconds(1)
//                                .lineTo(new Vector2d(55, -10))
//                                .lineToLinearHeading(new Pose2d(-6, -15, 0))
//                                .waitSeconds(3)
//                                .lineTo(new Vector2d(-6, -36))
//                                .lineTo(new Vector2d(-12, -36))
//                                .waitSeconds(1)
//                                .lineTo(new Vector2d(-34, -36))
//                                .lineTo(new Vector2d(-6, -15))
//                                .waitSeconds(2)
//                                .forward(10)
//                                .build()
//
//                );
//
//        Pose2d secondBotStartPose = new Pose2d(-63, -47, Math.PI);
//
//        Constraints secondBotConstraints = new Constraints(55, 60, Math.toRadians(211.05948602103072), Math.toRadians(211.05948602103072), 15.7471307087);
//        RoadRunnerBotEntity secondBot = new RoadRunnerBotEntity(mm, secondBotConstraints, 17.75, 17.75, secondBotStartPose, new ColorSchemeBlueDark(), 1.0);
//
//        TrajectorySequence secondBotTs = secondBot.getDrive().trajectorySequenceBuilder(secondBotStartPose)
//                .setReversed(true)
//                .addTemporalMarker(0.1, () -> {})
//                .waitSeconds(16)
//                .splineTo(new Vector2d(-1,-59), 0) //wobble
//                .addTemporalMarker(1, ()->{}) //time to wait before dropping intake
//                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {})
//                .UNSTABLE_addTemporalMarkerOffset(1.25, ()-> {})
//                .UNSTABLE_addTemporalMarkerOffset(2.49, ()-> {})
//                .waitSeconds(2.5) //total time for wobble sequence
//                .lineTo(shooterVector) //shooting
//                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {})
//                .UNSTABLE_addTemporalMarkerOffset(3.25, ()-> {})
//                .UNSTABLE_addTemporalMarkerOffset(4.9, ()-> {})
//                .waitSeconds(5) //total time for shooting sequence
//                .splineToLinearHeading(new Pose2d(10, -38, Math.toRadians(30)), 0) //park
//                .addTemporalMarker(0.9, 0.1, ()-> {})
//                .build();
//
//        secondBot.followTrajectorySequence(secondBotTs);
//        secondBot.start();
//
//        mm.requestToAddEntity(secondBot);
//
//        mm.start();
//    }
//}

package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTesting{
    public static Pose2d startPose = new Pose2d(-63, -47, Math.PI); //starting pose
    public static Vector2d shooterVector = new Vector2d(-8,-55); //shooting vector
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 60, Math.toRadians(211.05948602103072), Math.toRadians(211.05948602103072), 15.7471307087)
                .followTrajectorySequence(drive ->     drive.trajectorySequenceBuilder(new Pose2d(-63, 47, Math.PI))
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-11,58, Math.toRadians(160)), 0) //shooting
                        .waitSeconds(5)
                        .splineToSplineHeading(new Pose2d(40, 56, Math.toRadians(225)), 0)
                        .waitSeconds(2.5)
                        .setReversed(false)
                        .splineTo(new Vector2d(10, 50), Math.PI)
                        .splineTo(new Vector2d(-12, 36), Math.PI) //spline to stack
                        .forward(16)
                        .waitSeconds(7)
                        .forward(12)
                        .waitSeconds(6)
                        .lineToLinearHeading(new Pose2d(10, 58, -Math.PI/2))
                        .build()

                )
                .start();
    }
}

//
///*
//    blue 0 stack
//*/
///*
//    blue 1 stack
//    drive.trajectorySequenceBuilder(new Pose2d(-63.25, 54, Math.PI))
//    .setReversed(true)
//    .splineToLinearHeading(new Pose2d(-8,56, Math.toRadians(160)), 0) //shooting
//    .waitSeconds(1)
//    .splineToConstantHeading(new Vector2d(24, 48), 0) //wobble
//    .waitSeconds(1)
//    .lineToLinearHeading(new Pose2d(10,58, Math.toRadians(-90))) //park
//    .build()
//*/
///*
//    blue 4 stack
//    drive.trajectorySequenceBuilder(new Pose2d(-63.25, 54, Math.PI))
//    .setReversed(true)
//    .splineToLinearHeading(new Pose2d(-8,56, Math.toRadians(160)), 0) //shooting
//    .waitSeconds(1)
//    .lineToSplineHeading(new Pose2d(48, 56, Math.toRadians(225))) //wobble
//    .waitSeconds(1)
//    .lineToSplineHeading(new Pose2d(4,56, Math.toRadians(-60))) //park
//    .build()
//*/
///*
//    blue 4 stack; stack pickup
//     drive.trajectorySequenceBuilder(new Pose2d(-63.25, 54, Math.PI))
//    .setReversed(true)
//    .splineToLinearHeading(new Pose2d(-8,56, Math.toRadians(160)), 0) //shooting
//    .waitSeconds(1)
//    .lineToSplineHeading(new Pose2d(48, 56, Math.toRadians(225))) //wobble
//    .waitSeconds(1)
//    .lineToSplineHeading(new Pose2d(-20, 42, Math.toRadians(225)))
//    .waitSeconds(1)
//    .lineTo(new Vector2d(-26,36))
//    .waitSeconds(1)
//    .lineToSplineHeading(new Pose2d(4,56, Math.toRadians(-60))) //park
//    .build()
//*/
//

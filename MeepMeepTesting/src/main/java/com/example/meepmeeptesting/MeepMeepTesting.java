package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTesting {
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
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-63.25, 54, Math.PI))
                                .setReversed(true)
                                .addTemporalMarker(0.1, () -> {})
                                .addTemporalMarker(1, ()->{})
                                .lineToLinearHeading(new Pose2d(1,56, Math.toRadians(220)))
                                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {})
                                .UNSTABLE_addTemporalMarkerOffset(1.25, ()-> {})
                                .UNSTABLE_addTemporalMarkerOffset(2.49, ()-> {})
                                .waitSeconds(2.5)
                                .splineToLinearHeading(new Pose2d(-8,56, Math.PI), 0) //shooting
                                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {})
                                .UNSTABLE_addTemporalMarkerOffset(3.25, ()-> {})
                                .UNSTABLE_addTemporalMarkerOffset(4.9, ()-> {})
                                .waitSeconds(5)
                                .lineToLinearHeading(new Pose2d(-40,56, Math.PI))
                                .waitSeconds(13)
                                .splineToLinearHeading(new Pose2d(14, 36, Math.toRadians(1)), 0)
                                .addTemporalMarker(0.9, 0.1, ()->{})
                                .build()
                )
                .start();
    }
}

/*
    blue 0 stack
*/
/*
    blue 1 stack
    drive.trajectorySequenceBuilder(new Pose2d(-63.25, 54, Math.PI))
    .setReversed(true)
    .splineToLinearHeading(new Pose2d(-8,56, Math.toRadians(160)), 0) //shooting
    .waitSeconds(1)
    .splineToConstantHeading(new Vector2d(24, 48), 0) //wobble
    .waitSeconds(1)
    .lineToLinearHeading(new Pose2d(10,58, Math.toRadians(-90))) //park
    .build()
*/
/*
    blue 4 stack
    drive.trajectorySequenceBuilder(new Pose2d(-63.25, 54, Math.PI))
    .setReversed(true)
    .splineToLinearHeading(new Pose2d(-8,56, Math.toRadians(160)), 0) //shooting
    .waitSeconds(1)
    .lineToSplineHeading(new Pose2d(48, 56, Math.toRadians(225))) //wobble
    .waitSeconds(1)
    .lineToSplineHeading(new Pose2d(4,56, Math.toRadians(-60))) //park
    .build()
*/
/*
    blue 4 stack; stack pickup
     drive.trajectorySequenceBuilder(new Pose2d(-63.25, 54, Math.PI))
    .setReversed(true)
    .splineToLinearHeading(new Pose2d(-8,56, Math.toRadians(160)), 0) //shooting
    .waitSeconds(1)
    .lineToSplineHeading(new Pose2d(48, 56, Math.toRadians(225))) //wobble
    .waitSeconds(1)
    .lineToSplineHeading(new Pose2d(-20, 42, Math.toRadians(225)))
    .waitSeconds(1)
    .lineTo(new Vector2d(-26,36))
    .waitSeconds(1)
    .lineToSplineHeading(new Pose2d(4,56, Math.toRadians(-60))) //park
    .build()
*/


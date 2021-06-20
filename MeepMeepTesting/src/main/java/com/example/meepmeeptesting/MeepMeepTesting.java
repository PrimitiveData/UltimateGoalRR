package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTesting {
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
                        drive.trajectorySequenceBuilder(new Pose2d(-63.25, -54, Math.PI))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-6, -58), 0)
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(24, -37), 0)
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(10,-58, Math.toRadians(90)))
                        .build()
                )
                .start();
    }
}

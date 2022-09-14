package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.io.IOException;
import java.net.URL;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        DefaultBotBuilder blueBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(180), 13)
                .setDimensions(15, 15);

        RoadRunnerBotEntity blueEntity = blueBot.followTrajectorySequence(sixCycleBlue(blueBot.build().getDrive()));

        try {
            meepMeep.setBackground(ImageIO.read(new URL("https://i.imgur.com/mHV90qU.png")))
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(blueEntity)
                    .start();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static TrajectorySequence sixCycleBlue(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(-34, 64, Math.toRadians(90)))
                .setReversed(true)
                .back(48)
                .splineToSplineHeading(new Pose2d(-32, 6, Math.toRadians(150)), Math.toRadians(-30))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-32, 6, Math.toRadians(150)), Math.toRadians(-30))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-32, 6, Math.toRadians(150)), Math.toRadians(-30))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-32, 6, Math.toRadians(150)), Math.toRadians(-30))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-32, 6, Math.toRadians(150)), Math.toRadians(-30))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-32, 6, Math.toRadians(150)), Math.toRadians(-30))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .strafeRight(24)
                .build();
    }
}
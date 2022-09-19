package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.net.URL;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        DefaultBotBuilder blueBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15, 15);

        RoadRunnerBotEntity blueEntity = blueBot.followTrajectorySequence(sixCycleBlueLittleMovement(blueBot.build().getDrive()));

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueEntity)
                .start();
    }

    private static TrajectorySequence sixCycleBlue(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(-34, 64, Math.toRadians(90)))
                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-30, 8), Math.toRadians(-45))
                .setReversed(false)
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-8, 18, Math.toRadians(210)), Math.toRadians(30))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-8, 18, Math.toRadians(210)), Math.toRadians(30))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-8, 18, Math.toRadians(210)), Math.toRadians(30))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-8, 18, Math.toRadians(210)), Math.toRadians(30))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-8, 18, Math.toRadians(210)), Math.toRadians(30))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(-90)), Math.toRadians(-90))
                .back(24)
                //.strafeRight(48)
                .build();
    }

    private static TrajectorySequence sixCycleBlueLittleMovement(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(-34, 64, Math.toRadians(90)))
                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-33, 12), Math.toRadians(-45))
                .setReversed(false)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(180)), Math.toRadians(180))
                .back(18)
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 15, Math.toRadians(210)), Math.toRadians(30))
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-18, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 15, Math.toRadians(210)), Math.toRadians(30))
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-18, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 15, Math.toRadians(210)), Math.toRadians(30))
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-18, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 15, Math.toRadians(210)), Math.toRadians(30))
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-18, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 15, Math.toRadians(210)), Math.toRadians(30))
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(-90)), Math.toRadians(-90))
                .back(24)
                .strafeRight(48)
                .build();
    }


}
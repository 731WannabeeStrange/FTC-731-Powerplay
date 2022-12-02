package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        DefaultBotBuilder blueBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(16, 16);

        RoadRunnerBotEntity blueEntity = blueBot.followTrajectorySequence(onePlusTen(blueBot.build().getDrive()));

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueEntity)
                .start();
    }

    private static TrajectorySequence scrimmageAuto(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(-34, 64, Math.toRadians(90)))
                .back(54)
                .waitSeconds(0.5)
                .turn(Math.toRadians(45))
                .waitSeconds(1)
                .forward(3)
                .waitSeconds(0.5)
                .turn(Math.toRadians(45))
                .forward(24)
                .waitSeconds(1)
                .back(24)
                .waitSeconds(0.5)
                .turn(Math.toRadians(-45))
                .waitSeconds(0.5)
                .back(3)
                .waitSeconds(1)
                .forward(3)
                .waitSeconds(0.5)
                .turn(Math.toRadians(45))
                .forward(24)
                .waitSeconds(1)
                .back(24)
                .turn(Math.toRadians(-45))
                .waitSeconds(0.5)
                .back(3)
                .waitSeconds(1)
                .forward(3)
                .waitSeconds(0.5)
                .turn(Math.toRadians(45))
                .waitSeconds(0.5)
                .back(24)
                .build();
    }

    private static TrajectorySequence sixCycleBlue(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(-34, 64, Math.toRadians(90)))
                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-30, 8), Math.toRadians(-45))
                .setReversed(false)
                .waitSeconds(0.3)
                .splineTo(new Vector2d(-58, 12), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineTo(new Vector2d(-16, 15), Math.toRadians(25))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineTo(new Vector2d(-58, 12), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineTo(new Vector2d(-16, 15), Math.toRadians(25))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineTo(new Vector2d(-58, 12), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineTo(new Vector2d(-16, 15), Math.toRadians(25))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineTo(new Vector2d(-58, 12), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineTo(new Vector2d(-16, 15), Math.toRadians(25))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineTo(new Vector2d(-58, 12), Math.toRadians(180))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineTo(new Vector2d(-16, 15), Math.toRadians(25))
                .waitSeconds(0.3)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(-90)), Math.toRadians(-90))
                .back(24)
                //.strafeRight(48)
                .build();
    }

    private static TrajectorySequence onePlusTen(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(-34, 64, Math.toRadians(90)))
                .back(36)
                .splineToSplineHeading(new Pose2d(-30, 12, Math.toRadians(180)), Math.toRadians(0))
                .back(18)
                .waitSeconds(12)
                .turn(Math.toRadians(180))
                .forward(24)
                .waitSeconds(10)
                .forward(46)
                .build();
    }


}
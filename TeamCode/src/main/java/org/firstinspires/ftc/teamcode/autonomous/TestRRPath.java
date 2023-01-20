package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class TestRRPath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence path = drive.trajectorySequenceBuilder(startPose)
                .back(36)
                .splineToSplineHeading(new Pose2d(-30, 12, Math.toRadians(180)), Math.toRadians(0))
                .back(6)
                .waitSeconds(3)
                .back(12)
                .waitSeconds(3)
                .forward(12)
                .waitSeconds(3)
                .forward(12)
                .waitSeconds(3)
                .back(12)
                .waitSeconds(3)
                .forward(36)
                .waitSeconds(3)
                .back(36)
                .build();

        waitForStart();


        if (opModeIsActive()) {
            drive.followTrajectorySequence(path);
        }
    }
}

package org.firstinspires.ftc.teamcode.autonomous.commandbased;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.commandbased.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.groups.ScoreCone;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.signal.AprilTagVisionPipeline;
import org.firstinspires.ftc.teamcode.vision.signal.Location;

public class CommandRightAuto extends LinearOpMode {
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LiftSubsystem liftSubsystem;

    private AprilTagVisionPipeline pipeline;
    private Location location = Location.LEFT;

    private Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(90));

    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    public void run() {
        CommandScheduler.getInstance().run();
    }

    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(hardwareMap, startPose);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        liftSubsystem = new LiftSubsystem(hardwareMap);

        register(driveSubsystem, intakeSubsystem, liftSubsystem);

        pipeline = new AprilTagVisionPipeline();

        TrajectorySequence driveToSpot = driveSubsystem.trajectorySequenceBuilder(startPose)
                .back(36)
                .splineToSplineHeading(new Pose2d(-30, 12.5, Math.toRadians(180)), Math.toRadians(0))
                .back(4)
                .waitSeconds(1)
                .build();

        TrajectorySequence leftPark = driveSubsystem.trajectorySequenceBuilder(driveToSpot.end())
                .lineTo(new Vector2d(-12, 12))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence midPark = driveSubsystem.trajectorySequenceBuilder(driveToSpot.end())
                .lineTo(new Vector2d(-36, 12))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence rightPark = driveSubsystem.trajectorySequenceBuilder(driveToSpot.end())
                .lineTo(new Vector2d(-60, 12))
                .turn(Math.toRadians(90))
                .build();

        while (opModeInInit()) {
            location = pipeline.visionLoop(telemetry);
        }

        schedule(new FollowTrajectory(driveSubsystem, driveToSpot));

        while (!isStopRequested() && opModeIsActive()) {
            run();
        }

        reset();
    }
}

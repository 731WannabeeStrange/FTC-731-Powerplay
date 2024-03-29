package org.firstinspires.ftc.teamcode.autonomous.commandbased;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.commandbased.commands.DepositCone;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.commands.GoToLiftState;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.commands.ResetLiftAndIntake;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.groups.ScoreCone;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.signal.AprilTagVisionPipeline;
import org.firstinspires.ftc.teamcode.vision.signal.Location;

import java.util.HashMap;

@Autonomous
public class CommandRightAuto extends LinearOpMode {
    private CommandScheduler scheduler;

    private DriveSubsystem driveSubsystem;
    private Intake intakeSubsystem;
    private Lift liftSubsystem;

    private AprilTagVisionPipeline pipeline = new AprilTagVisionPipeline();
    private Location location = Location.LEFT;

    private final Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(90));

    private MultipleTelemetry multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    private TimerTrigger timerTrigger;
    private boolean parking = false;

    @Override
    public void runOpMode() throws InterruptedException {
        scheduler = CommandScheduler.getInstance();

        driveSubsystem = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new Intake(hardwareMap, multipleTelemetry);
        liftSubsystem = new Lift(hardwareMap, multipleTelemetry);

        driveSubsystem.setPoseEstimate(startPose);

        scheduler.registerSubsystem(driveSubsystem, intakeSubsystem, liftSubsystem);

        pipeline.init(hardwareMap, telemetry);

        TrajectorySequence driveToSpot = driveSubsystem.trajectorySequenceBuilder(startPose)
                .back(36)
                .splineToSplineHeading(new Pose2d(-28, 12, Math.toRadians(180)), Math.toRadians(0))
                .back(2)
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

        GoToLiftState retractCommand = new GoToLiftState(liftSubsystem, Lift.LiftState.RETRACT);
        liftSubsystem.setDefaultCommand(retractCommand);

        while (opModeInInit()) {
            location = pipeline.visionLoop(telemetry);
        }

        scheduler.schedule(new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new FollowTrajectory(driveSubsystem, driveToSpot),
                        new GoToLiftState(liftSubsystem, Lift.LiftState.GOING_UP)),
                new ScoreCone(intakeSubsystem, liftSubsystem, -90, Lift.LiftState.HIGH, 0.35),
                new ScoreCone(intakeSubsystem, liftSubsystem, -90, Lift.LiftState.HIGH, 0.30),
                new DepositCone(liftSubsystem, Lift.LiftState.HIGH, -90),
                new ParallelCommandGroup(
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(Location.LEFT, new FollowTrajectory(driveSubsystem, leftPark));
                                    put(Location.MIDDLE, new FollowTrajectory(driveSubsystem, midPark));
                                    put(Location.RIGHT, new FollowTrajectory(driveSubsystem, rightPark));
                                }},
                                () -> location
                        ),
                        new ResetLiftAndIntake(intakeSubsystem, liftSubsystem)
                )
        ));

        timerTrigger = new TimerTrigger(getRuntime());

        while (!isStopRequested() && opModeIsActive()) {
            scheduler.run();
            multipleTelemetry.update();

            /*
            timerTrigger.whenActive(new ParallelCommandGroup(
                    new SelectCommand(
                            new HashMap<Object, Command>() {{
                                put(Location.LEFT, new FollowTrajectory(driveSubsystem, leftPark));
                                put(Location.MIDDLE, new FollowTrajectory(driveSubsystem, midPark));
                                put(Location.RIGHT, new FollowTrajectory(driveSubsystem, rightPark));
                            }},
                            () -> location
                    ),
                    new ResetLiftAndIntake(intakeSubsystem, liftSubsystem)
            ));
             */
        }

        scheduler.reset();
    }
}
